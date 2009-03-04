#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <gccore.h>
#include <wiiuse/wpad.h>

#define DEFAULT_FIFO_SIZE	(256*1024)
#include <ode/ode.h>

static void *frameBuffer[2] = { NULL, NULL};
GXRModeObj *rmode;

Mtx GXmodelView2D;

enum orientation {TOP, BOTTOM, LEFT, RIGHT, FRONT, BACK};
enum axis_direction {PITCH_PLUS, PITCH_MIN, YAW_PLUS, YAW_MIN, ROLL_PLUS, ROLL_MIN};

enum orientation orientation_down = BOTTOM; //The side of the Wiimote facing down to the ground
enum orientation orientation_down_old = BOTTOM;
enum orientation orientation_front = RIGHT; //The side of the Wiimote facing the television
enum axis_direction horizontal_axis;
enum axis_direction vertical_axis;
int grav = 1;
		
struct QUAT oldquat;
float x, y, z, x_n, y_n, z_n;
gforce_t gforce;

#define PI 3.141592653589f

float yaw = 0, pitch = 0, roll = 0;

Mtx _mtxelements[32]; //max stack depth is 32 in opengl
int _mtxcurrentelement;
void  glPopMatrix (Mtx model){
	_mtxcurrentelement--;
	guMtxCopy(_mtxelements[_mtxcurrentelement], model);
}

void  glPushMatrix (Mtx model){
	guMtxCopy(model, _mtxelements[_mtxcurrentelement]);
	_mtxcurrentelement++;
}

enum orientation opposite(enum orientation o)
{
	switch(o)
	{
		case TOP: return BOTTOM;
		case BOTTOM: return TOP;
		case LEFT: return RIGHT;
		case RIGHT: return LEFT;
		case FRONT: return BACK;
		case BACK: return FRONT;
	}
	return BACK;
}

struct QUAT {
	float w, x, y, z;
};

void QuatSlerp(struct QUAT * from, struct QUAT * to, float t, struct QUAT * res)
      {
        float           to1[4];
        double        omega, cosom, sinom, scale0, scale1;


        // calc cosine
        cosom = from->x * to->x + from->y * to->y + from->z * to->z
			       + from->w * to->w;


        // adjust signs (if necessary)
        if ( cosom <0.0 ){ cosom = -cosom; to1[0] = - to->x;
		to1[1] = - to->y;
		to1[2] = - to->z;
		to1[3] = - to->w;
        } else  {
		to1[0] = to->x;
		to1[1] = to->y;
		to1[2] = to->z;
		to1[3] = to->w;
        }


        // calculate coefficients

		//const double DELTA = 0.9995;
		const double DELTA = 0.0005;
       if ( (1.0 - cosom) > DELTA ) {
                // standard case (slerp)
                omega = acos(cosom);
                sinom = sin(omega);
                scale0 = sin((1.0 - t) * omega) / sinom;
                scale1 = sin(t * omega) / sinom;


        } else {        
    // "from" and "to" quaternions are very close 
	    //  ... so we can do a linear interpolation
                scale0 = 1.0 - t;
                scale1 = t;
        }
	// calculate final values
	res->x = scale0 * from->x + scale1 * to1[0];
	res->y = scale0 * from->y + scale1 * to1[1];
	res->z = scale0 * from->z + scale1 * to1[2];
	res->w = scale0 * from->w + scale1 * to1[3];
}

void QuatToMatrix(struct QUAT * quat, float m[4][4]){

    float wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;


    // calculate coefficients
    x2 = quat->x + quat->x; y2 = quat->y + quat->y;
    z2 = quat->z + quat->z;
    xx = quat->x * x2; xy = quat->x * y2; xz = quat->x * z2;
    yy = quat->y * y2; yz = quat->y * z2; zz = quat->z * z2;
    wx = quat->w * x2; wy = quat->w * y2; wz = quat->w * z2;


    m[0][0] = 1.0 - (yy + zz); m[1][0] = xy - wz;
    m[2][0] = xz + wy; m[3][0] = 0.0;

    m[0][1] = xy + wz; m[1][1] = 1.0 - (xx + zz);
    m[2][1] = yz - wx; m[3][1] = 0.0;


    m[0][2] = xz - wy; m[1][2] = yz + wx;
    m[2][2] = 1.0 - (xx + yy); m[3][2] = 0.0;


    m[0][3] = 0; m[1][3] = 0;
    m[2][3] = 0; m[3][3] = 1;

}

void EulerToQuat(float roll, float pitch, float yaw, struct QUAT* quat)
{
	float cr, cp, cy, sr, sp, sy, cpcy, spsy;


// calculate trig identities
cr = cos(roll/2);

	cp = cos(pitch/2);
	cy = cos(yaw/2);


	sr = sin(roll/2);
	sp = sin(pitch/2);
	sy = sin(yaw/2);
	
	cpcy = cp * cy;
	spsy = sp * sy;


	quat->w = cr * cpcy + sr * spsy;
	quat->x = sr * cpcy - cr * spsy;
	quat->y = cr * sp * cy + sr * cp * sy;
	quat->z = cr * cp * sy - sr * sp * cy;
}

dWorldID world;
dJointGroupID contactGroup;
dGeomID cube;

// Callback function for collision detection
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 4;

	dContact contact[N];
	int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

	int i;
    for (i = 0; i < n; i++) {
		contact[i].surface.mode = dContactBounce;
		contact[i].surface.mu = 50;
		contact[i].surface.bounce = 0.1;
		contact[i].surface.bounce_vel = 0.1;
    

      dJointID c = dJointCreateContact(world,contactGroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
    }
   
 }



void updateinput()
{
		WPAD_ScanPads();
		WPAD_GForce(0, &gforce);
		if (WPAD_ButtonsDown(0) & WPAD_BUTTON_HOME) exit(0);

		x = gforce.x;
		y = gforce.y;
		z = gforce.z;
		//x += (gforce.x - x) / 10;
		//y += (gforce.y - y) / 10;
		//z += (gforce.z - z) / 10;
		
		/** Length of the vector. */
		const float xyz_len = sqrt(x*x + y*y + z*z);
		/* Normalize vector. */
		x_n = x / xyz_len;
		y_n = y / xyz_len;
		z_n = z / xyz_len;
		
		//
		float smallest = 400.0f; // infinity
		if(fabs(1 - x_n) < smallest)
		{
			smallest = fabs(1 - x_n);
			orientation_down = RIGHT;
		}
		if(fabs(1 + x_n) < smallest)
		{
			smallest = fabs(1 + x_n);
			orientation_down = LEFT;
		}
		
		if(fabs(1 - y_n) < smallest)
		{
			smallest = fabs(1 - y_n);
			orientation_down = FRONT;
		}
		if(fabs(1 + y_n) < smallest)
		{
			smallest = fabs(1 + y_n);
			orientation_down = BACK;
		}
		if(fabs(1 - z_n) < smallest)
		{
			smallest = fabs(1 - z_n);
			orientation_down = BOTTOM;
		}
		if(fabs(1 + z_n) < smallest)
		{
			smallest = fabs(1 + z_n);
			orientation_down = TOP;
		}		
}

//---------------------------------------------------------------------------------
int main( int argc, char **argv ){
//---------------------------------------------------------------------------------
	f32 yscale;

	u32 xfbHeight;

	Mtx view;
	Mtx44 perspective;
	Mtx model, modelview;


	u32	fb = 0; 	// initial framebuffer index
	GXColor background = {0, 0, 0, 0xff};
	

	// init the vi.
	VIDEO_Init();
	WPAD_Init();
	WPAD_SetDataFormat(0,WPAD_FMT_BTNS_ACC_IR);
	WPAD_SetVRes(0,640,480);
 
	rmode = VIDEO_GetPreferredMode(NULL);
	
	// allocate 2 framebuffers for double buffering
	frameBuffer[0] = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));
	frameBuffer[1] = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));

	VIDEO_Configure(rmode);
	VIDEO_SetNextFramebuffer(frameBuffer[fb]);
	VIDEO_SetBlack(FALSE);
	VIDEO_Flush();
	VIDEO_WaitVSync();
	if(rmode->viTVMode&VI_NON_INTERLACE) VIDEO_WaitVSync();

	// setup the fifo and then init the flipper
	void *gp_fifo = NULL;
	gp_fifo = memalign(32,DEFAULT_FIFO_SIZE);
	memset(gp_fifo,0,DEFAULT_FIFO_SIZE);
 
	GX_Init(gp_fifo,DEFAULT_FIFO_SIZE);
 
	// clears the bg to color and clears the z buffer
	GX_SetCopyClear(background, 0x00ffffff);
 
	// other gx setup
	GX_SetViewport(0,0,rmode->fbWidth,rmode->efbHeight,0,1);
	yscale = GX_GetYScaleFactor(rmode->efbHeight,rmode->xfbHeight);
	xfbHeight = GX_SetDispCopyYScale(yscale);
	GX_SetScissor(0,0,rmode->fbWidth,rmode->efbHeight);
	GX_SetDispCopySrc(0,0,rmode->fbWidth,rmode->efbHeight);
	GX_SetDispCopyDst(rmode->fbWidth,xfbHeight);
	GX_SetCopyFilter(rmode->aa,rmode->sample_pattern,GX_TRUE,rmode->vfilter);
	GX_SetFieldMode(rmode->field_rendering,((rmode->viHeight==2*rmode->xfbHeight)?GX_ENABLE:GX_DISABLE));
 
	GX_SetCullMode(GX_CULL_BACK);
	GX_CopyDisp(frameBuffer[fb],GX_TRUE);
	GX_SetDispCopyGamma(GX_GM_1_0);
 

	// setup the vertex descriptor
	// tells the flipper to expect direct data
	GX_ClearVtxDesc();
	GX_SetVtxDesc(GX_VA_POS, GX_DIRECT);
 	GX_SetVtxDesc(GX_VA_CLR0, GX_DIRECT);
 
	// setup the vertex attribute table
	// describes the data
	// args: vat location 0-7, type of data, data format, size, scale
	// so for ex. in the first call we are sending position data with
	// 3 values X,Y,Z of size F32. scale sets the number of fractional
	// bits for non float data.
	GX_SetVtxAttrFmt(GX_VTXFMT0, GX_VA_POS, GX_POS_XYZ, GX_F32, 0);
	GX_SetVtxAttrFmt(GX_VTXFMT0, GX_VA_CLR0, GX_CLR_RGBA, GX_RGB8, 0);
 
	GX_SetNumChans(1);
	GX_SetNumTexGens(0);
	GX_SetTevOrder(GX_TEVSTAGE0, GX_TEXCOORDNULL, GX_TEXMAP_NULL, GX_COLOR0A0);
	GX_SetTevOp(GX_TEVSTAGE0, GX_PASSCLR);

	// setup our camera at the origin
	// looking down the -z axis with y up
	Vector cam = {0.0F, 0.0F, 5.0F},
			up = {0.0F, 1.0F, 0.0F},
		  look = {0.0F, 0.0F, 0.0F};
	guLookAt(view, &cam, &up, &look);
 

	// setup our projection matrix
	// this creates a perspective matrix with a view angle of 90,
	// and aspect ratio based on the display resolution
    f32 w = rmode->viWidth;
    f32 h = rmode->viHeight;
	guPerspective(perspective, 45, (f32)w/h, 0.1F, 300.0F);
	GX_LoadProjectionMtx(perspective, GX_PERSPECTIVE);

	dInitODE2(0);
	world = dWorldCreate();
	contactGroup = dJointGroupCreate(0);

	
	dSpaceID space = dSimpleSpaceCreate(0);

/*                            z
                              |
	    Top                   |___x  
							 /
        4----5              y
	   /|   /|
Left  6----7 |  Right      Back
	  | 0--|-1             /
	  |/   |/             /
      2----3           Front
	
	  Bottom
*/
		const dVector3 Vertices[] = {
			{-1.0, -1.0, -1.0},	// 0
			{1.0, -1.0, -1.0},	// 1
			{-1.0, 1.0, -1.0},	// 2
			{1.0, 1.0, -1.0},	// 3
			{-1.0, -1.0, 1.0},	// 4
			{1.0, -1.0, 1.0},	// 5
			{-1.0, 1.0, 1.0},	// 6
			{1.0, 1.0, 1.0},	// 7	
		};

		const unsigned int Indices[] = {
			0, 1, 2,
			3, 2, 1,	//Bottom
			
			6, 5, 4,
			7, 5, 6,	//Top			
			
			0, 2, 4,
			2, 6, 4,	//Left

			3, 5, 7,
			3, 1, 5,	//Right
			
			3, 7, 2,
			2, 7, 6,	//Front

			0, 4, 5,
			1, 0, 5,	//Back
		};

        dTriMeshDataID cubedata = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildSimple(cubedata,
                                    (dReal*)Vertices,
                                    8,
                                    Indices,
                                    3 * 2 * 6);
        //cube = dCreateTriMesh(space, cubedata, 0, 0, 0);
		
		dGeomID box_bottom = dCreateBox(space, 2,2,2);
		dGeomSetPosition(box_bottom, 0, 0, -2);
		dGeomSetBody(box_bottom, 0);

		dGeomID box_top = dCreateBox(space, 2,2,2);
		dGeomSetPosition(box_top, 0, 0, 2);
		dGeomSetBody(box_top, 0);
		
		dGeomID box_left = dCreateBox(space, 2,2,2);
		dGeomSetPosition(box_left, 0, -2, 0);
		dGeomSetBody(box_left, 0);
		
		dGeomID box_right = dCreateBox(space, 2,2,2);
		dGeomSetPosition(box_right, 0, 2, 0);
		dGeomSetBody(box_right, 0);
		
		dGeomID box_front = dCreateBox(space, 2,2,2);
		dGeomSetPosition(box_front, 2, 0, 0);
		dGeomSetBody(box_front, 0);
		
		dGeomID box_back = dCreateBox(space, 2,2,2);
		dGeomSetPosition(box_back, -2, 0, 0);
		dGeomSetBody(box_back, 0);
		
		
		dBodyID body = dBodyCreate(world);
		dBodySetPosition(body,0,0,0);
		
		dMass mass;
		dMassSetBox(&mass,1,1,1,1);
		dMassAdjust(&mass,2.0f);
		dBodySetMass(body,&mass);
		
		

		dGeomID sphere = dCreateSphere(space, 0.15);
		//dGeomID sphere = dCreateBox(space, 0.4, 0.2, 0.4);
		dGeomSetBody(sphere, body);

	while(1) {
		updateinput();
		// do this before drawing
		GX_SetViewport(0,0,rmode->fbWidth,rmode->efbHeight,0,1);
		guMtxIdentity(model);
	
		if(orientation_down != orientation_down_old)
		{
			if(orientation_down == orientation_front) {
				orientation_front = opposite(orientation_down_old);
			} else if(orientation_down == opposite(orientation_front)) {
				orientation_front = orientation_down_old;
			}
			orientation_down_old = orientation_down;
		}
		
		float angle_roll = 0,
			  angle_pitch = 0,
			  angle_yaw = 0;

	/* GRAVITY
	 * y axis: front of the controller corresponds to y = +1; back to y = -1;
	 * x axis: left of the controller corresponds to x = -1; right to x = +1;
	 * z axis: bottom of the controler corresponds to z = +1; top to z = -1;
	 */
	switch (orientation_front) {
		case FRONT:
			switch (orientation_down) {
				case FRONT:
				case BACK:
					break;
				case BOTTOM:
					angle_roll = -atanf(x / -z);
					angle_pitch = atanf(y / -z);
					break;
				case TOP:
					angle_roll = PI - atanf(x / -z);
					angle_pitch = -atanf(y / -z);
					break;
				case LEFT:
					angle_roll = -PI / 2 + atanf(-z / x);
					angle_pitch = atanf(y / x);
					break;
				case RIGHT:
					angle_roll = PI / 2 + atanf(-z / x);
					angle_pitch = -atanf(y / x);
					break;
			}
			angle_yaw = 0;
			break;
		case BACK:
			switch (orientation_down) {
				case FRONT:
				case BACK:
					break;
				case BOTTOM:
					angle_roll += atanf(x / -z);
					angle_pitch += -atanf(y / -z);
					break;
				case TOP:
					angle_roll += -PI + atanf(x / -z);
					angle_pitch += atanf(y / -z);
					break;
				case LEFT:
					angle_roll += PI / 2 - atanf(-z / x);
					angle_pitch += -atanf(y / x);
					break;
				case RIGHT:
					angle_roll += -PI / 2 - atanf(-z / x);
					angle_pitch += atanf(y / x);
					break;
			}
			angle_yaw = PI;
			break;
			
		case LEFT:
			switch (orientation_down) {
				case LEFT:
				case RIGHT:
					break;
				case BOTTOM:
					angle_roll += -atanf(y / -z);
					angle_pitch += -atanf(x / -z);
					break;
				case TOP:
					angle_roll += -PI - atanf(y / -z);
					angle_pitch += atanf(x / -z);
					break;
				case FRONT:
					angle_roll += PI / 2 + atanf(z / -y);
					angle_pitch +=  -atanf(x / -y);
					break;
				case BACK:
					angle_roll += -PI / 2 + atanf(z / -y);
					angle_pitch += atanf(x / -y);
					break;
			}
			angle_yaw = PI / 2;
			break;
			
		case RIGHT:
			switch (orientation_down) {
				case LEFT:
				case RIGHT:
					break;
				case BOTTOM:
					angle_roll += atanf(y / -z);
					angle_pitch += atanf(x / -z);
					break;
				case TOP:
					angle_roll += PI + atanf(y / -z);
					angle_pitch += -atanf(x / -z);
					break;
				case FRONT:
					angle_roll += -PI / 2 - atanf(z / -y);
					angle_pitch +=  atanf(x / -y);
					break;
				case BACK:
					angle_roll += PI / 2 - atanf(z / -y);
					angle_pitch += -atanf(x / -y);
					break;
			}
			angle_yaw = -PI / 2;
			break;
			
		case BOTTOM:
			angle_pitch = PI / 2;
			switch (orientation_down) {
				case TOP:
				case BOTTOM:
					break;
				case LEFT:
					angle_yaw += -PI / 2 - atanf(y / -x);
					angle_pitch -= atanf(z / -x);
					break;
				case RIGHT:
					angle_yaw += PI / 2 - atanf(y / -x);
					angle_pitch += atanf(z / -x);
					break;
				case FRONT:
					angle_yaw += PI + atanf(x / -y);
					angle_pitch += atanf(z / -y);
					break;
				case BACK:
					angle_yaw += atanf(x / -y);
					angle_pitch -= atanf(z / -y);
					break;
			}
			break;
		case TOP:
			angle_pitch = -PI / 2;
			switch (orientation_down) {
				case TOP:
				case BOTTOM:
					break;
				case LEFT:
					angle_yaw = PI / 2 - atanf(y / -x);
					angle_pitch += atanf(z / -x);
					break;
				case RIGHT:
					angle_yaw = -PI / 2 - atanf(y / -x);
					angle_pitch -= atanf(z / -x);
					break;
				case FRONT:
					angle_yaw = atanf(x / -y);
					angle_pitch -= atanf(z / -y);
					break;
				case BACK:
					angle_yaw = PI + atanf(x / -y);
					angle_pitch += atanf(z / -y);
					break;
			}
			break;
			
			default:
				break;
	}

		if(isnan(angle_pitch)) angle_pitch = 0;
		if(isnan(angle_roll)) angle_roll = 0;
		if(isnan(angle_yaw)) angle_yaw = 0;

//		angle_pitch = 0;
//		angle_roll = 0;
//		angle_yaw = 0;

		Mtx m;
		struct QUAT current, result;
		//EulerToQuat(angle_roll, angle_pitch, angle_yaw, &current);
		EulerToQuat(-angle_pitch, -angle_roll, angle_yaw, &current);
		QuatSlerp(&oldquat, &current, 0.2f, &result);
		QuatToMatrix(&result, m);
		oldquat = result;

		guMtxConcat ( m,model,model );
		
		guMtxConcat(view,model,modelview);
		
		// load the modelview matrix into matrix memory
		GX_LoadPosMtxImm(modelview, GX_PNMTX0);
		//glPushMatrix(model);
			float c1, c2, c3; 
		

	
		
		GX_Begin(GX_QUADS, GX_VTXFMT0, 24);			// Draw a Cube

		

			//ORANGE	FRONT
			c1 = 1.0f; c2 = 0.5f; c3 = 0.0f;
			GX_Position3f32( 1.0f, 1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f, 1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f, 1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 1.0f, 1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);

			//RED		BACK
			c1 = 1.0f; c2 = 0.0f; c3 = 0.0f;
			GX_Position3f32( 1.0f,-1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f,-1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f,-1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 1.0f,-1.0f,-1.0f);
			GX_Color3f32(1.0f,0.5f,0.0f);

			//WHITE		TOP
			c1 = 1.0f; c2 = 1.0f; c3 = 1.0f;
			GX_Position3f32( 1.0f, 1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f, 1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f,-1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 1.0f,-1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);

			//YELLOW	BOTTOM
			c1 = 1.0f; c2 = 1.0f; c3 = 0.0f;
			GX_Position3f32( 1.0f,-1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f,-1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f, 1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 1.0f, 1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);

			//GREEN		LEFT
			c1 = 0.0f; c2 = 1.0f; c3 = 0.0f;
			GX_Position3f32(-1.0f, 1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f, 1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f,-1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-1.0f,-1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);

			//BLUE		RIGHT
			c1 = 0.0f; c2 = 0.0f; c3 = 1.0f;
			GX_Position3f32( 1.0f, 1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 1.0f, 1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 1.0f,-1.0f, 1.0f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 1.0f,-1.0f,-1.0f);
			GX_Color3f32(c1, c2, c3);

		GX_End();									// Done Drawing The Quad 

		//glPopMatrix(model);

		//dMatrix3 cuberot;
		//dRFromEulerAngles (cuberot, -angle_pitch, -angle_roll, 0);
		//dGeomSetRotation(cube, cuberot);
		
		/*
		dQuaternion q;
		q[0] = result.x;
		q[1] = result.y;
		q[2] = result.z;
		q[3] = result.w;
		dGeomSetQuaternion (cube, q);
*/

		if (WPAD_ButtonsDown(0) & WPAD_BUTTON_B) {
			grav *= -1;
		}
		
		dWorldSetGravity(world, x*grav*10, y*grav*10, z*grav*-10);

		if (WPAD_ButtonsDown(0) & WPAD_BUTTON_A) {
			dMatrix3 RR;
			dRSetIdentity(RR);
			dBodySetPosition(body, 0, 0, 0);
			dBodySetLinearVel(body, 0, 0, 0);
			dBodySetAngularVel(body, 0, 0, 0);
			dBodySetRotation(body, RR);
		}
		
		dSpaceCollide(space, 0, &nearCallback);
		dWorldQuickStep(world, 0.06f);
		dJointGroupEmpty(contactGroup);
		
	
		const dReal *p = dBodyGetPosition(body);
		const dReal *R = dBodyGetRotation(body);

		Mtx44 matrix;
		guMtxIdentity(matrix);
		matrix[0][0]=R[0];
		matrix[0][1]=R[4];
		matrix[0][2]=R[8];
		
		matrix[1][0]=R[1];
		matrix[1][1]=R[5];
		matrix[1][2]=R[9];
	
		matrix[2][0]=R[2];
		matrix[2][1]=R[6];
		matrix[2][2]=R[10];
		guMtxInverse(matrix, matrix);
		
		guMtxIdentity(model);
		guMtxConcat (matrix,model,model);
		
		guMtxTransApply ( model, model, p[0], p[1], p[2] ); 
		guMtxConcat ( m,model,model );
		
		guMtxConcat(view,model,modelview);
		GX_LoadPosMtxImm(modelview, GX_PNMTX0);
		
		GX_Begin(GX_QUADS, GX_VTXFMT0, 24);			// Draw a Cube
			//ORANGE	FRONT
			c1 = 1.0f; c2 = 0.5f; c3 = 0.0f;
			c1 /= 2; c2 /= 2; c3 /= 2;
			
			GX_Position3f32( 0.1f, 0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f, 0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f, 0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 0.1f, 0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);


			//RED		BACK
			c1 = 1.0f; c2 = 0.0f; c3 = 0.0f;
			c1 /= 2; c2 /= 2; c3 /= 2;
			GX_Position3f32( 0.1f,-0.1f,-0.1f);
			GX_Color3f32(0.1f,0.5f,0.0f);
			GX_Position3f32(-0.1f,-0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f,-0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 0.1f,-0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			
			//WHITE		TOP
			c1 = 1.0f; c2 = 1.0f; c3 = 1.0f;
			c1 /= 2; c2 /= 2; c3 /= 2;
			GX_Position3f32( 0.1f,-0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f,-0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f, 0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 0.1f, 0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			
			//YELLOW	BOTTOM
			c1 = 1.0f; c2 = 1.0f; c3 = 0.0f;
			c1 /= 2; c2 /= 2; c3 /= 2;
			
			GX_Position3f32( 0.1f, 0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f, 0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f,-0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 0.1f,-0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			
			//GREEN		LEFT
			c1 = 0.0f; c2 = 1.0f; c3 = 0.0f;
			c1 /= 2; c2 /= 2; c3 /= 2;
			GX_Position3f32(-0.1f,-0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f,-0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f, 0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32(-0.1f, 0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);

			//BLUE		RIGHT
			c1 = 0.0f; c2 = 0.0f; c3 = 1.0f;
			c1 /= 2; c2 /= 2; c3 /= 2;
			GX_Position3f32( 0.1f,-0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 0.1f,-0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 0.1f, 0.1f, 0.1f);
			GX_Color3f32(c1, c2, c3);
			GX_Position3f32( 0.1f, 0.1f,-0.1f);
			GX_Color3f32(c1, c2, c3);
		GX_End();									// Done Drawing The Quad 
		
		// do this stuff after drawing
		GX_DrawDone();
		
		fb ^= 1;		// flip framebuffer
		GX_SetZMode(GX_TRUE, GX_LEQUAL, GX_TRUE);
		GX_SetColorUpdate(GX_TRUE);
		GX_CopyDisp(frameBuffer[fb],GX_TRUE);

		VIDEO_SetNextFramebuffer(frameBuffer[fb]);
 
		VIDEO_Flush();
 
		VIDEO_WaitVSync();
	}
	return 0;
}
 
