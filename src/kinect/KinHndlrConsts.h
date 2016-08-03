/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): John Turner <jturner65@gatech.edu>
 * Georgia Tech Graphics Lab
 * 
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * This code incorporates portions of Open Dynamics Engine 
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights 
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow 
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __KINHNDLRCONSTS_H__
#define __KINHNDLRCONSTS_H__

#include "KinLibHelper.h"

namespace rtql8 {
	namespace kinect{
		////////////////////////
		//KinectHandler/General
		////////////////////////

		const double PI					= 3.14159265359;							//...
		//consts defined in NUI code, using smaller, more friendly variable names
		const int NUI_NumSkels			= NUI_SKELETON_COUNT;						//number of potentially tracked skeletons/skel pos (currently 6, up to 2 tracked skels, the rest are tracked skel pos
		const int NUI_NumJnts			= NUI_SKELETON_POSITION_COUNT;				//number of potentially tracked skel joints.  if this changes the bone and joint values below need to be updated
		const int NUI_NumBones			= NUI_NumJnts - 1;							//true for all skeletons so long as they are trees and have no cycles

		// Use to define 0 vecs and matrices
		#define vec3_zero (Eigen::Vector3d(0, 0, 0))
		#define vec4_zero (Eigen::Vector4d(0, 0, 0, 0))
		#define mat3_zero (Eigen::Matrix3d::Zero())
		#define mat4_zero (Eigen::Matrix4d::Zero())	
	
        const int strmImgW = 640;								//constants defining kinect camera's image dimensions
        const int strmImgH = 480;

		const double KH_AUD_ANGLE_FAIL = -9999;					//failure code for audio angle data retrieval

		//state flag idx's used by kinecthandler's state machine
		const int _KH_SEATED_IDX			= 0;				//set seated mode
		const int _KH_NEAR_IDX				= 1;				//set near mode

		const int _KH_SKEL_TRACKED_IDX		= 2;				//a skeleton is currently tracked
		const int _KH_SKEL_ROOTONLY_IDX		= 3;				//best skeleton currently tracked is only root
		const int _KH_SKEL_NONEFOUND_IDX	= 4;				//no skeleton currently tracked

		const int _KH_SKH_VLD_JNTL_IDX		= 5;				//skelhandler has valid joint locations to send to avatar
		const int _KH_SKH_AJNT_INIT_IDX		= 6;				//avatar joint vals have been inited in kinect handler
		const int _KH_SKH_AJNT_SET_IDX		= 7;				//avatar joint vals have been sent to skel hndlr

		const int _KH_VLD_IMAGE_STRM		= 8;				//valid image stream exists
		const int _KH_VLD_DEPTH_STRM		= 9;				//valid depth stream exists
		const int _KH_VLD_SKEL_STRM			= 10;				//valid skel stream exists
		const int _KH_VLD_NTRCT_STRM		= 11;				//valid interaction stream exists
		const int _KH_VLD_AUDIO_STRM		= 12;				//valid audio stream exists

		const int _KH_DPTH_DISP_HND			= 13;				//display hands in depth stream image
		const int _KH_SKEL_CALC_HND			= 14;				//calculate hand positions from stream

		const int _KH_DISP_DEBUG			= 15;				//display debug Output

		const int KH_NumFlags				= 16;				//number of boolean flags


        //idx's of various handlers
        const int _G_KinHndlr               = 0;
        const int _G_KinAUHndlr             = 1;
        const int _G_KinDPHndlr             = 2;
        const int _G_KinIMHndlr             = 3;
        const int _G_KinINHndlr             = 4;
        const int _G_KinSKHndlr             = 5;
        const int _G_KinCntrlr              = 6;
        const int _G_KinKUIHndlr            = 7;

        const int G_NumHndlrs               = 8;

        extern int _RCD;									//reconnection countdown
        extern long kinMaxTilt;								//potentially change with new kinect hardware
        extern long kinMinTilt;

		const double kinHNDL_epsVal = .000001;

		//names of joints in NUI enum, minus prefix
		const string kinEnumJointNames[] = {"HIP_CENTER", "SPINE",	"SHOULDER_CENTER", "HEAD", "SHOULDER_LEFT",	"ELBOW_LEFT", "WRIST_LEFT", "HAND_LEFT", "SHOULDER_RIGHT", "ELBOW_RIGHT",	
														"WRIST_RIGHT", "HAND_RIGHT", "HIP_LEFT", "KNEE_LEFT", "ANKLE_LEFT", "FOOT_LEFT", "HIP_RIGHT", "KNEE_RIGHT", "ANKLE_RIGHT",	
														"FOOT_RIGHT"};
		//names of joints fit for printing, using enum idx
		const string kinSkelJointNames[] = {"Root", "Spine", "Center Shoulder", "Head", 
												"Left Shoulder","Left Elbow", "Left Wrist", "Left Hand", "Right Shoulder", "Right Elbow", "Right Wrist", "Right Hand",	
												"Left Hip", "Left Knee", "Left Ankle", "Left Foot", "Right Hip", "Right Knee", "Right Ankle","Right Foot"};

		//names of joints fit for printing, using enum idx
		const string kinSkelBoneNames[] = {"Left Foot", "Left Shin", "Left Thigh", "Right Foot", "Right Shin", "Right Thigh", "Neck", "Left Hand", "Left Forearm", "Left Bicep", "Left Shoulder",
												"Right Hand", "Right Forearm", "Right Bicep", "Right Shoulder", "Upper Spine", "Lower Spine", "Right Hip", "Left Hip"};
	
		////////////////////////
		//end KinectHandler/General
		////////////////////////

		////////////////////////
		//KinectController/ KinCntrlHand
		////////////////////////

		#define KIN_IK_SKEL_NAME RTQL8_DATA_PATH"skel/fullbody1Coach.skel"			                    //filename for skeleton to be used to IK in data to
		//#define KIN_IK_SKEL_NAME RTQL8_DATA_PATH"skel/fullbody5_sphere.skel"			                    //filename for skeleton to be used to IK in data to - sehoon's
        #define KIN_CNT_DEFAULT_CONFIG_XML RTQL8_DATA_PATH"xml/defKinCntrlrConfig.xml"                  //filename for default configuration of KinectController setup and gui interface
        extern int TRY_KIN_RCN;												                //how long to wait until reconnecting
			//constants denoting src of events
		const int KC_LEFT = 0;
		const int KC_RIGHT = 1;
        const int KC_MOUSE = 2;

		//kinect controller state machine flags
		const int _KC_AVTR_INIT_SET			= 0;									//avatar's bone dimensions have been set upon initial entry into skel handler
			//stream flags : use the following flags to optionally enable or disable stream functionalities - streams will still be implemented if necessary dependencies
		const int _KC_USE_STRM_DPTH			= 1;									//use data from a particular stream - depth
		const int _KC_USE_STRM_RGB			= 2;									//use data from a particular stream - RGB image
		const int _KC_USE_STRM_SKEL			= 3;									//use data from a particular stream - skeleton (for IK)
		const int _KC_USE_STRM_AUD			= 4;									//use data from a particular stream - audio
		const int _KC_USE_STRM_INTR			= 5;									//use data from a particular stream - interaction (gripping hands)

			//display flags : which images and constructs to display
		const int _KC_DISP_DEPTH			= 6;									//display depth image
		const int _KC_DISP_RGB				= 7;									//display rgb image
		const int _KC_DISP_AVATAR			= 8;									//display avatar model in draw window
		const int _KC_DISP_MRKRS			= 9;									//display marker locations on avatar skeleton in draw window
		const int _KC_DISP_AVTLOC			= 10;									//display avatar post-IK joint locs
		const int _KC_DISP_RAWSK			= 11;									//display raw skeleton joints and bones from kinect skeleton stream
		const int _KC_DISP_FILTSK			= 12;									//display filtered skeleton joints and bones from kinect skeleton stream
		const int _KC_DISP_SKCLIP			= 13;									//display skel stream clipping box
		const int _KC_DISP_UI_2D			= 14;									//display 2d UI overlay in 2d interaction mode
		const int _KC_DISP_UI_3D			= 15;									//display 3d UI overlay in 3d interaction mode
		const int _KC_DISP_UI_IK			= 16;									//display UI overlay during IK mode

			//action/functionality flags : turn on or off certain processing/modes
		const int _KC_PROC_2D_DGRAB     	= 17;									//process/display 2d hands grab functionality built in depth handler
            //only 1 of the following should ever be true at any 1 time
		const int _KC_PROC_HANDS_2D			= 18;									//process/display 2d hands to interact with gui layer - turn on 2d UI interaction/display, turn off other modes
		const int _KC_PROC_HANDS_3D			= 19;									//process/display 3d hands to interact with skel/scene objs
		const int _KC_PROC_IK				= 20;									//process IK - want to turn off when we are using hands, freezing coach skel

        const int _KC_PROC_XML_FLAGS        = 21;                                   //default flags for all streams and handlers have been set from xml, and should be sent to where they belong once kinecthandler and stream handlers have been instantiated

		const int _KC_MOUSE_USE				= 22;									//mouse is currently being used - toggle on based on mouse movement or off based on hand movement

        const int KC_numFlags				= 23;									//number of boolean flags for skel handler sm - 1 higher than last flag

        extern float HndTrkBxMult[6];
        extern float KinPshDist;												    //arbitrary amount denoting how far forward a push is.
        extern int KC_hndEps_2D;									                //amount hands need to move before considered "moved", in x-y dimensions
        extern float LHandDragSens; 
        extern float RHandDragSens; 
        extern float LHandPushSensIn;                                               //type:float      cmt:left hand push in sensitivity to detect click
        extern float RHandPushSensIn;                                               //type:float      cmt:left hand push in sensitivity to detect click
        extern float LHandPushSensOut;                                              //type:float      cmt:right hand pull back sensitivity to disengage click
        extern float RHandPushSensOut;                                              //type:float      cmt:right hand pull back sensitivity to disengage click
        extern int randSfx;

        //hand flag idx's
		const int _KHnd_HAND_CHNG_2D		= 0;									//2d ui either hand has changed state or position
		const int _KHnd_HAND_DRAG_OBJ	    = 1;									//hand is dragging valid UI object - locked to position, don't look inside other object while drag engaged
		const int _KHnd_HAND_IN_OBJ		    = 2;									//hand is in valid UI object    
		const int _KHnd_HAND_CLCKD		    = 3;									//hand has already generated a click event - to prevent multiple clicks being registered- should be cleared on transition from click to open  
        const int _KHnd_HAND_TRACKED        = 4;                                    //if the hand is tracked or not

        const int KHnd_numFlags             = 5;                                    //number of flag idx's

        //hand image consts
		const int KinNmPlmPts = 10;
		const int KinNmOFPts = 6;
		const int KinNmCFPts = 4;
		const float pMult[KinNmPlmPts][2] = {{.5f,1.75f}, {1.25f,1.45f} , {1.75f, .8f}, {1.75f, -.8f}, {1.0f,-1.75f}, {-1.0f,-1.75f}, {-1.75f,-.5f}, {-1.65f,.5f}, {-1.1f, 1.0f}, {-.5f,1.75f}};
		const float ofMult[5][KinNmOFPts][2] = {
			{{pMult[6][0],pMult[6][1]},	{pMult[7][0],pMult[7][1]}, {pMult[7][0] - .75f,pMult[7][1]},		{pMult[7][0] - .25f,pMult[7][1] + .5f},		{pMult[7][0] - .75f, pMult[7][1] + 1.05f},	{pMult[7][0] - .25f,pMult[7][1] + 1.5f}},				//thumb
			{{pMult[8][0],pMult[8][1]},	{pMult[9][0],pMult[9][1]}, {pMult[8][0] - .25f,pMult[8][1] + 1.25f},{pMult[8][0] + .25f,pMult[8][1] + 2.25f},	{pMult[8][0] - .15f,pMult[8][1] + 2.75f},	{pMult[8][0] + .15f,pMult[8][1] + 3.25f}},				//index
			{{pMult[9][0],pMult[9][1]},	{pMult[0][0],pMult[0][1]}, {pMult[9][0] + .2f,pMult[9][1] + 1.5f},	{pMult[0][0] - .2f,pMult[0][1] + 2.5f},		{pMult[9][0] + .3f,pMult[9][1] + 2.75f},	{pMult[0][0] - .3f,pMult[0][1] + 3.25f}},				//middle
			{{pMult[0][0],pMult[0][1]},	{pMult[1][0],pMult[1][1]}, {pMult[0][0] + .2f,pMult[0][1] + 1.5f},	{pMult[1][0], pMult[1][1] + 2.5f},			{pMult[0][0] + .3f,pMult[0][1] + 2.75f},	{pMult[1][0] - .1f,pMult[1][1] + 3.35f}},				//ring
			{{pMult[2][0],pMult[2][1]},	{pMult[1][0],pMult[1][1]}, {pMult[2][0] + .4f,pMult[2][1] + 1.5f},	{pMult[2][0], pMult[2][1] + 2.5f},			{pMult[2][0] + .5f,pMult[2][1] + 2.75f},	{pMult[2][0] + .3f,pMult[2][1] + 3.35f}}};				//pinkie

		const float cfMult[5][KinNmCFPts][2] = {
			{{pMult[6][0],pMult[6][1]},	{pMult[7][0],pMult[7][1]},{pMult[7][0] - .15f,pMult[7][1] + .25f},	{pMult[7][0] + .5f,	pMult[7][1] + .45f}},				//thumb
			{{pMult[8][0]-.15f,pMult[8][1]-.25f},	{pMult[9][0],pMult[9][1]},{pMult[8][0]-.25f ,pMult[8][1] + .55f},  {pMult[9][0] - .25f,pMult[9][1] + .15f}},	//index
			{{pMult[9][0],pMult[9][1]},	{pMult[0][0],pMult[0][1]},{pMult[9][0] + .2f,pMult[9][1] + .25f},	{pMult[0][0] - .2f,pMult[0][1] + .25f}},				//middle
			{{pMult[0][0],pMult[0][1]},	{pMult[1][0],pMult[1][1]},{pMult[0][0] + .2f,pMult[0][1] + .15f},	{pMult[1][0], pMult[1][1] + .225f}},					//ring
			{{pMult[1][0],pMult[1][1]},	{pMult[2][0],pMult[2][1]},{pMult[1][0] +.2f ,pMult[1][1] + .125f},	{pMult[2][0] + .05f, pMult[2][1] + .25f}}};				//pinkie
				
			//pushBar progress dims
		const float pshBarlen = 100;
		const float dims[][2] = {{-7,15}, {7,15}};

		////////////////////////
		//end KinectController
		////////////////////////

		////////////////////////
		//KinUIComponents
		////////////////////////
            //immediate draw flags
		const int UIobjIDX_Display		= 0;						//display the ui object
        const int UIobjIDX_DispHS       = 1;                        //display hotspot around object
        const int UIobjIDX_DispMO       = 2;                        //display mouseover popuptext
        const int UIobjIDX_OffsLbl      = 3;                        //offset label for sliders and cranks
            //capability flags
        const int UIobjIDX_CanClick		= 4;						//object is clickable
		const int UIobjIDX_CanDrag		= 5;						//object is draggable
        const int UIobjIDX_CanDispMO    = 6;                        //mouse over text is enabled for this object
        const int UIobjIDX_CanType      = 7;                        //object accepts/requires specifically keyboard input from user to change data (text box, listbox w/string values (?))
            //immediate action flags
		const int UIobjIDX_MSDown		= 8;						//mouse/hand has pressed on object
		const int UIobjIDX_MSUp			= 9;						//mouse/hand has release object
		const int UIobjIDX_MSDrag		= 10;						//mouse/hand is dragging object					   
            //event related flags
        const int UIobjIDX_Click        = 11;                        //click event recognized
            //debug
        const int UIobjIDX_DispDebug    = 12;                       //display debug info - what interface (mouse, lhand, rhand) caused event

		const int UIobj_numFlags        = 13;

		//for event value return - idx into object's value array - based on event change return value
		const int UIobj_numVals = 3;

		//border around slider object
        extern float KinSldrBrdr;
		//sensitivity of slider/crank dragging - should be 0.0 - 1.0
        extern float UIobj_DragSens;
        extern float UIobj_CrankSens;   
        //font display values - min and max scaling value
        extern float UIobj_MinFontScale;
        extern float UIobj_MaxFontScale;

		////////////////////////
		//KinDepthHandler
		////////////////////////

		#define KIN_HDLR_DPTHALPHA (0xff)
        extern double NNalpha;
		const int _DP_SEATED_IDX		= 0;
		const int _DP_NEAR_IDX			= 1;
		const int _DP_PROC_HNDS			= 2;				        //process hand location/grip state
		const int _DP_DISP_HNDS			= 3;				        //display hands in depth image
		const int _DP_BUILD_HNDIMG		= 4;				        //build hand images from processed depth data

        const int _DP_RECORD_NN         = 5;                        //start recording information from hand positions
        const int _DP_USE_NN            = 6;                        //use currently loaded NN to classify hand postures among 2 possibilities - open and grip

		const int DP_NumFlags = 7;

        extern float DP_GRIPH_THRESH;                               //type:float      cmt: threshold plane for grip detection - height
        extern float DP_GRIPW_THRESH;                               //type:float      cmt: threshold plane for grip detection - width
        extern float DP_GRIPD_THRESH;                               //type:float      cmt: threshold plane for grip detection - depth


			//depth stream initialization values
		const NUI_IMAGE_TYPE dpthInitImgType	= NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX;					//NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX needed for interactions
																											//use NUI_IMAGE_TYPE_COLOR_INFRARED for infrared camera
		const NUI_IMAGE_RESOLUTION dpthInitRes	= NUI_IMAGE_RESOLUTION_640x480;								//depth cam resolution - will change with future kinects
		const DWORD dpthInitFlags				= NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES;		//flags for depth image
		const DWORD dpthInitFrameBuf			= NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM;						//buffer for depth frames

			//hand tracking box
        extern int LHandBx[4];				//LHandBx = array of vals for left hand track box = {max x, min x-max x, max y, min y-max y}
        extern int RHandBx[4];				//RHandBx = array of vals for right hand track box = {max x, min x-max x, max y, min y-max y}

		////////////////////////
		//end KinDepthHandler
		////////////////////////

		////////////////////////
		//KinImageHandler
		////////////////////////

		#define KIN_HDLR_RGBALPHA (0xff)

		const int _IM_SEATED_IDX		= 0;
		const int _IM_NEAR_IDX			= 1;

		const int IM_NumFlags = 2;

			//rgb stream initialization values
		const NUI_IMAGE_TYPE rgbInitImgType		= NUI_IMAGE_TYPE_COLOR;						//rgb image type
		const NUI_IMAGE_RESOLUTION rgbInitRes	= NUI_IMAGE_RESOLUTION_640x480;				//rgb cam resolution - will change with future kinects
		const DWORD rgbInitFlags				= 0;										//flags for rgb image
		const DWORD rgbInitFrameBuf				= 2;										//buffer for rgb frames - suggested in sdk

		////////////////////////
		//end KinImageHandler
		////////////////////////

		////////////////////////
		//KinSkelHandler
		////////////////////////

		//skelHandler flags idx's
		const int _SK_SEATED_IDX			= 0;
		const int _SK_NEAR_IDX				= 1;
		const int _SK_AVTR_INIT_SET			= 2;									//avatar's bone dimensions have been set upon initial entry into skel handler
		const int _SK_DISP_AVTR_SKEL		= 3;									//display avatar "bone and joint" skeleton (not ik avatar)
		const int _SK_DISP_FLTR_SKEL		= 4;									//display kinect filtered skeleton bone and joints (avatar is ik'ed to this)
		const int _SK_DISP_NSY_SKEL			= 5;									//display skel built from raw joint data direct from kinect
		const int _SK_CAM_TILT_SET			= 6;									//new skeleton tracked, cam tilt set to handle it
		const int _SK_INT_CROUCH			= 7;									//interpolate unknown knee joint data for crouching
		const int _SK_SAVE_JDATA			= 8;									//save joint data to file for testing purposes without kinect present

		const int SK_numFlags				= 9;									//number of boolean flags for skel handler sm - 1 higher than last flag

		const int NumLimbs				= 5;										//head, 2 hands, 2 feet
		const int NumBnsLmbPth			= 6;										//size of ara of bones from root to each limb tip
		const int SK_MSG_DISP			= 90;										//# of iters to wait between msgs to console/log regarding skeleton tracking status
		const double SK_MV_THRESH		= .15;										//threshold for distance root can move in skel space before recalculating root location (for height to determine kneeling)
		const double SK_STND_THRESH		= .05;										//distance to move vertically to decide whether or not to recalculate staning root location
		const double SK_CRCH_THRESH		= .97;										//crouch theshold - % root can lower before crouch calculation is initiated for knee location
		
        extern NUI_TRANSFORM_SMOOTH_PARAMETERS skelSmoothing;		                    //used for kinect sdk's internal calc of skeleton motion smoothing - see sdk for params
        extern int MX_CNT_FLR_Y;										                //max number of consecutive frames of same floor depth counted (if kinect's interpolated floor changes, decrement until 0 then change, to act as smoothing)
        extern float SK_JNT_MULT[3];						                                //multiplier used when converting joint coords from skel space to display space	
        extern float SK_JNT_OFF[3];						                                //offset used when converting joint coords from skel space to display space	

		const float boneClr[][4] = {{0.0f,0.0f,0.0f,1.0f},							//colors for noisy skel bones (black), conditioned by handler bones, avatar bones.  last entry is inferred(both joints) bones
									{1.0f,0.0f,0.0f,1.0f},							//conditioned bones are red
									{0.0f,1.0f,1.0f,1.0f},							//avatar bone bars are cyan 
									{1.0f,1.0f,1.0f,1.0f}};							//inferred bones are white

		const double jntClr[][3] = {{1.0f,1.0f,0.0f},								//colors for noisy skel joints (yellow), conditioned by handler joints, avatar joints.  last entry is inferred joints
									{0.0f,1.0f,0.0f},								//conditioned joints are green
									{1.0f,0.0f,1.0f},								//avatar joints are magenta 
									{0.0f,0.0f,1.0f}};								//inferred joints are blue
		const int infClrIDX = 3;													//idx in color arrays for inferred joints

        extern double jntSz[3];										                //size of skel joint for display
		//joint smoothing/interpolation constants
        extern int jntPosAraDpth;												//how many previous values for jnt pos,vel,accel we will keep around
        extern int jntNumMaxInterp;												//maximum interpolations before we just use most recent avatar value for joint location

		const int SK_NSY_KIN = 0;
		const int SK_FLT_KIN = 1;
		const int SK_AVTR = 2;								//skeleton types, for drawing : noisy, filtered, avatar	
	
		//idxs of bones in array holding bone vectors - use NUISensor-defined enum aliases for joints - build/vector direction order for bones is from root to tip
		//what kinect calls left and what avatar denotes as left are opposite due to mirroring
		const int bIdx_AnkleL_FootL			= 0;			//	_leftFootLimbIDX		
		const int bIdx_KneeL_AnkleL			= 1;			//	_leftShinLimbIDX			
		const int bIdx_HipL_KneeL			= 2;			//	_leftThighLimbIDX		
		const int bIdx_AnkleR_FootR			= 3;			//	_rightFootLimbIDX			
		const int bIdx_KneeR_AnkleR			= 4;			//	_rightShinLimbIDX			
		const int bIdx_HipR_KneeR			= 5;			//	_rightThighLimbIDX			
		const int bIdx_ShldrC_Head			= 6;			//	_neckLimbIDX			
		const int bIdx_WristL_HandL			= 7;			//	_leftHandLimbIDX			
		const int bIdx_ElbowL_WristL		= 8;			//	_leftForearmLimbIDX		
		const int bIdx_ShldrL_ElbowL		= 9;			//	_leftBicepLimbIDX			
		const int bIdx_ShldrC_ShldrL		= 10;			//	_leftScapulaLimbIDX	
		const int bIdx_WristR_HandR			= 11;			//	_rightHandLimbIDX		
		const int bIdx_ElbowR_WristR		= 12;			//	_rightForearmLimbIDX	
		const int bIdx_ShldrR_ElbowR		= 13;			//	_rightBicepLimbIDX			
		const int bIdx_ShldrC_ShldrR		= 14;			//	_rightScapulaLimbIDX	
		const int bIdx_Spine_ShldrC			= 15;			//	_spineLimbIDX			
		const int bIdx_HipC_Spine			= 16;			//	_abdomenLimbIDX		
		const int bIdx_HipC_HipR			= 17;			//	_rightHipLimbIDX			
		const int bIdx_HipC_HipL			= 18;			//	_leftHipLimbIDX		

	
		//alias NuiSensor joint idx enum with less cumbersome names
		const int jIdx_HipC		= NUI_SKELETON_POSITION_HIP_CENTER;
		const int jIdx_Spine	= NUI_SKELETON_POSITION_SPINE;
		const int jIdx_ShldrC	= NUI_SKELETON_POSITION_SHOULDER_CENTER;
		const int jIdx_Head		= NUI_SKELETON_POSITION_HEAD;
		const int jIdx_ShldrL	= NUI_SKELETON_POSITION_SHOULDER_LEFT;
		const int jIdx_ElbowL	= NUI_SKELETON_POSITION_ELBOW_LEFT;
		const int jIdx_WristL	= NUI_SKELETON_POSITION_WRIST_LEFT;
		const int jIdx_HandL	= NUI_SKELETON_POSITION_HAND_LEFT;
		const int jIdx_ShldrR	= NUI_SKELETON_POSITION_SHOULDER_RIGHT;
		const int jIdx_ElbowR	= NUI_SKELETON_POSITION_ELBOW_RIGHT;
		const int jIdx_WristR	= NUI_SKELETON_POSITION_WRIST_RIGHT;
		const int jIdx_HandR	= NUI_SKELETON_POSITION_HAND_RIGHT;
		const int jIdx_HipL		= NUI_SKELETON_POSITION_HIP_LEFT;
		const int jIdx_KneeL	= NUI_SKELETON_POSITION_KNEE_LEFT;
		const int jIdx_AnkleL	= NUI_SKELETON_POSITION_ANKLE_LEFT;
		const int jIdx_FootL	= NUI_SKELETON_POSITION_FOOT_LEFT;
		const int jIdx_HipR		= NUI_SKELETON_POSITION_HIP_RIGHT;
		const int jIdx_KneeR	= NUI_SKELETON_POSITION_KNEE_RIGHT;
		const int jIdx_AnkleR	= NUI_SKELETON_POSITION_ANKLE_RIGHT;
		const int jIdx_FootR	= NUI_SKELETON_POSITION_FOOT_RIGHT;


		const int jIdx_ROOT = jIdx_HipC;								//index of root is idx of hip center, as defined in kinect docs

		//neighbor joints in both dirs - any joint that has only 1 neighbor uses itself for missing neighbor
			//neighbor joint toward root
		//jIdx_HipC is root
		const int jntPrev[] = {jIdx_HipC,								//jnt idx : 	jIdx_HipC		
								jIdx_HipC, 								//				jIdx_Spine		
								jIdx_Spine,								//				jIdx_ShldrC	
								jIdx_ShldrC,							//				jIdx_Head		
								jIdx_ShldrC,							//				jIdx_ShldrL	
								jIdx_ShldrL,							//				jIdx_ElbowL	
								jIdx_ElbowL,							//				jIdx_WristL	
								jIdx_WristL,							//				jIdx_HandL		
								jIdx_ShldrC,							//				jIdx_ShldrR	
								jIdx_ShldrR,							//				jIdx_ElbowR	
								jIdx_ElbowR,							//				jIdx_WristR	
								jIdx_WristR,							//				jIdx_HandR		
								jIdx_HipC,								//				jIdx_HipL		
								jIdx_HipL,								//				jIdx_KneeL		
								jIdx_KneeL,								//				jIdx_AnkleL	
								jIdx_AnkleL,							//				jIdx_FootL		
								jIdx_HipC,								//				jIdx_HipR		
								jIdx_HipR,								//				jIdx_KneeR		
								jIdx_KneeR,								//				jIdx_AnkleR	
								jIdx_AnkleR	};							//				jIdx_FootR		

			//neighbor joint toward extremity - root uses spine, spine 
			//uses shoulder center, shoulder center uses head, end-joints (head, hands, feet) use themselves
		const int jntNext[] = {jIdx_Spine,								//jnt idx : 	jIdx_HipC		
								jIdx_ShldrC, 							//				jIdx_Spine		
								jIdx_Head,								//				jIdx_ShldrC	
								jIdx_Head,								//				jIdx_Head		
								jIdx_ElbowL,							//				jIdx_ShldrL	
								jIdx_WristL,							//				jIdx_ElbowL	
								jIdx_HandL,								//				jIdx_WristL	
								jIdx_HandL,								//				jIdx_HandL		
								jIdx_ElbowR,							//				jIdx_ShldrR	
								jIdx_WristR,							//				jIdx_ElbowR	
								jIdx_HandR,								//				jIdx_WristR	
								jIdx_HandR,								//				jIdx_HandR		
								jIdx_KneeL,								//				jIdx_HipL		
								jIdx_AnkleL,							//				jIdx_KneeL		
								jIdx_FootL,								//				jIdx_AnkleL	
								jIdx_FootL,								//				jIdx_FootL		
								jIdx_KneeR,								//				jIdx_HipR		
								jIdx_AnkleR,							//				jIdx_KneeR		
								jIdx_FootR,								//				jIdx_AnkleR	
								jIdx_FootR	};							//				jIdx_FootR		

			//kinect uses mirroring for joints, so idx's of joints need to swap from left to right
		const int kinJnts[] = {jIdx_HipC,								//				jIdx_HipC		<-- joint on avatar matches this joint on kinect
							   jIdx_Spine,								//				jIdx_Spine	
							   jIdx_ShldrC,								//				jIdx_ShldrC	
							   jIdx_Head,								//				jIdx_Head		
							   jIdx_ShldrR,								//				jIdx_ShldrL	
							   jIdx_ElbowR,								//				jIdx_ElbowL	
							   jIdx_WristR,								//				jIdx_WristL	
							   jIdx_HandR,								//				jIdx_HandL	
							   jIdx_ShldrL,								//				jIdx_ShldrR	
							   jIdx_ElbowL,								//				jIdx_ElbowR	
							   jIdx_WristL,								//				jIdx_WristR	
							   jIdx_HandL,								//				jIdx_HandR	
							   jIdx_HipR,								//				jIdx_HipL		
							   jIdx_KneeR,								//				jIdx_KneeL	
							   jIdx_AnkleR,								//				jIdx_AnkleL	
							   jIdx_FootR,								//				jIdx_FootL	
							   jIdx_HipL,								//				jIdx_HipR		
							   jIdx_KneeL,								//				jIdx_KneeR	
							   jIdx_AnkleL,								//				jIdx_AnkleR	
							   jIdx_FootL};								//				jIdx_FootR	

			//the two joints that make up every bone
		const int bnJoints[][2] = {	{jIdx_FootL, jIdx_AnkleL},			//				bIdx_AnkleL_FootL		
									{jIdx_AnkleL, jIdx_KneeL},			//				bIdx_KneeL_AnkleL		
									{jIdx_KneeL, jIdx_HipL},			//				bIdx_HipL_KneeL		
									{jIdx_FootR, jIdx_AnkleR},			//				bIdx_AnkleR_FootR		
									{jIdx_AnkleR, jIdx_KneeR},			//				bIdx_KneeR_AnkleR		
									{jIdx_KneeR, jIdx_HipR},			//				bIdx_HipR_KneeR		
									{jIdx_Head, jIdx_ShldrC},			//				bIdx_ShldrC_Head		
									{jIdx_HandL, jIdx_WristL},			//				bIdx_WristL_HandL		
									{jIdx_WristL, jIdx_ElbowL},			//				bIdx_ElbowL_WristL	
									{jIdx_ElbowL, jIdx_ShldrL},			//				bIdx_ShldrL_ElbowL	
									{jIdx_ShldrL, jIdx_ShldrC},			//				bIdx_ShldrC_ShldrL	
									{jIdx_HandR, jIdx_WristR},			//				bIdx_WristR_HandR		
									{jIdx_WristR, jIdx_ElbowR},			//				bIdx_ElbowR_WristR	
									{jIdx_ElbowR, jIdx_ShldrR},			//				bIdx_ShldrR_ElbowR	
									{jIdx_ShldrR, jIdx_ShldrC},			//				bIdx_ShldrC_ShldrR	
									{jIdx_ShldrC, jIdx_Spine},			//				bIdx_Spine_ShldrC		
									{jIdx_Spine, jIdx_HipC},			//				bIdx_HipC_Spine		
									{jIdx_HipR, jIdx_HipC},				//				bIdx_HipC_HipR		
									{jIdx_HipL, jIdx_HipC}				//				bIdx_HipC_HipL		
							};

			//array of all joint idx's that are at ends of limbs
		const int jntTips[] = {jIdx_Head, jIdx_HandR, jIdx_HandL, jIdx_FootR, jIdx_FootL};
			//array of joint idxs from root to tips for each tip joint - -1 means done
		const int jntRtPthToTip[][7] = {{jIdx_HipC, jIdx_Spine, jIdx_ShldrC, jIdx_Head,-1, -1, -1},											//jntTips[0] = jIdx_Head			-- redundant paths ignored
										{jIdx_HipC, jIdx_Spine, jIdx_ShldrC, jIdx_ShldrR, jIdx_ElbowR, jIdx_WristR, jIdx_HandR},			//jntTips[1] = jIdx_HandR
										{jIdx_HipC, jIdx_Spine, jIdx_ShldrC, jIdx_ShldrL, jIdx_ElbowL, jIdx_WristL, jIdx_HandL},			//jntTips[2] = jIdx_HandL
										{jIdx_HipC, jIdx_HipR, jIdx_KneeR, jIdx_AnkleR, jIdx_FootR, -1, -1},								//jntTips[3] = jIdx_FootR
										{jIdx_HipC, jIdx_HipL, jIdx_KneeL, jIdx_AnkleL, jIdx_FootL, -1, -1},								//jntTips[4] = jIdx_FootL
									};

			//array of bn idx's for the path from root to each tip joint	
		const int bnRtPthToTip[][6] = { {bIdx_HipC_Spine, bIdx_Spine_ShldrC, bIdx_ShldrC_Head, -1, -1, -1},															//jntTips[0] = jIdx_Head			-- redundant paths ignored
										{bIdx_HipC_Spine, bIdx_Spine_ShldrC, bIdx_ShldrC_ShldrR, bIdx_ShldrR_ElbowR, bIdx_ElbowR_WristR, bIdx_WristR_HandR},		//jntTips[1] = jIdx_HandR
										{bIdx_HipC_Spine, bIdx_Spine_ShldrC, bIdx_ShldrC_ShldrL, bIdx_ShldrL_ElbowL, bIdx_ElbowL_WristL, bIdx_WristL_HandL},		//jntTips[2] = jIdx_HandL
										{bIdx_HipC_HipR, bIdx_HipR_KneeR, bIdx_KneeR_AnkleR, bIdx_AnkleR_FootR, -1, -1},											//jntTips[3] = jIdx_FootR
										{bIdx_HipC_HipL, bIdx_HipL_KneeL, bIdx_KneeL_AnkleL, bIdx_AnkleL_FootL, -1, -1}};											//jntTips[4] = jIdx_FootL

			//maximum displacement per timestep/iteration of skelframe reading for each joint  - used to scale noisy joint readings
			//possibly replace with array built on the fly
		const double jntMaxDisp[] = {.02,							//jnt idx : 		jIdx_HipC	
									.02,							//					jIdx_Spine	
									.05,							//					jIdx_ShldrC	
									.10,							//					jIdx_Head	
									.05,							//					jIdx_ShldrL	
									.10,							//					jIdx_ElbowL	
									.20,							//					jIdx_WristL	
									.30,							//					jIdx_HandL	
									.05,							//					jIdx_ShldrR	
									.10,							//					jIdx_ElbowR	
									.20,							//					jIdx_WristR	
									.30,							//					jIdx_HandR	
									.05,							//					jIdx_HipL	
									.10,							//					jIdx_KneeL	
									.20,							//					jIdx_AnkleL	
									.30,							//					jIdx_FootL	
									.05,							//					jIdx_HipR	
									.10,							//					jIdx_KneeR	
									.20,							//					jIdx_AnkleR	
									.30	};							//					jIdx_FootR	

        extern float kinSkelDisp[3][3];					//displacement for drawing raw kinect joint data, idx 0-noisy kin, 1-filtered kin, 2-avatar(postIK)

		////////////////////////
		//End KinSkelHandler
		////////////////////////

		////////////////////////
		//Start KinSkelIKSolver
		////////////////////////

        extern float KinIK_eps;	
        extern float KinIK_minPrtrb;				//minimum timestep amount we will allow for any single iteration
        extern int KinIK_numIters;					//number of IK iterations to run per timer cycle
        extern Eigen::VectorXd KinIK_weights;                              //type:Eigen::VectorXd    cmt:weights for joints in IK skel, idx'ed by kin joint ara idx - currently 20 joints in kin skel

		////////////////////////
		//End KinSkelIKSolver
		////////////////////////

		////////////////////////
		//Start KinInteractHandler/KinInteractClient
		////////////////////////
		const int _IH_SEATED_IDX			= 0;
		const int _IH_NEAR_IDX				= 1;
		const int _IH_SKEL_PROC				= 2;						//skeleton frame processed
		const int _IH_DPTH_PROC				= 3;						//depth frame processed

		const int IH_NumFlags = 4;						//# of interaction handler flags

		////////////////////////
		//End KinInteractHandler/KinInteractClient
		////////////////////////


		////////////////////////
		//Start KinAudioHandler
		////////////////////////
		const int _AU_SEATED_IDX			= 0;
		const int _AU_NEAR_IDX				= 1;

		const int AU_NumFlags = 2;						//# of audio handler flags

		const float Aud_ConfThresh = 0.3f;						//threshold of confidence required to register a hit
		const unsigned int NumBuffers = 20;						//# of audio buffers (chunks of overall circular buffer) used to capture Kinect audio data

        extern wstring GrammarFileName;                                     //use this file to hold primary grammar

		// Format of Kinect audio stream
		static const WORD       AudioFormat = WAVE_FORMAT_PCM;
		// Number of channels in Kinect audio stream
		static const WORD       AudioChannels = 1;
		// Samples per second in Kinect audio stream
		static const DWORD      AudioSamplesPerSecond = 16000;
		// Average bytes per second in Kinect audio stream
		static const DWORD      AudioAverageBytesPerSecond = 32000;
		// Block alignment in Kinect audio stream
		static const WORD       AudioBlockAlign = 2;
		// Bits per audio sample in Kinect audio stream
		static const WORD       AudioBitsPerSample = 16;

		#define INITGUID
		#include <guiddef.h>
		// This is the class ID we expect for the Microsoft Speech recognizer.
		// Other values indicate that we're using a version of sapi.h that is
		// incompatible with this sample.
		DEFINE_GUID(CLSID_ExpectedRecognizer, 0x495648e7, 0xf7ab, 0x4267, 0x8e, 0x0f, 0xca, 0xfb, 0x7a, 0x33, 0xc1, 0x60);


		// Safe release for interfaces - got from microsoft samples
		template<class Interface>
		inline void audSafeRelease( Interface *& pInterfaceToRelease ){
			if ( pInterfaceToRelease != NULL ){
				pInterfaceToRelease->Release();
				pInterfaceToRelease = NULL;
			}
		}
		////////////////////////
		//End KinAudioHandler
		////////////////////////

	}//namespace kinect
}//namespace rtql8 


#endif//__KINHNDLRCONSTS_H__
