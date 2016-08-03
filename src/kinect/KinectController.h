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

#ifndef __KINECTCONTROLLER_H__
#define __KINECTCONTROLLER_H__

#include "KinSkelIKSolver.h"
#include "KinectHandler.h"
#include "KinUIComponent.h"
#include "KinUIButton.h"
#include "KinUIImageButton.h"
#include "KinUISlider.h"
#include "KinUIPrgBar.h"
#include "KinUITimer.h"
#include "KinUICrank.h"
#include "KinUIPage.h"
#include "KinUICaption.h"
#include "KinCntrlHand.h"
#include "KinNNet.h"

#include "KinCntrlrXMLReader.h"

//this class should be inherited by a class wishing to consume the code in any application 
// wishing to use the library.  an instance of this inherited class should then be 
// instantiated to utilize kinect functionality, with the virtual methods overridden 
// to provide app-specific display and interaction functionality

namespace rtql8{
    namespace kinect{
        class KinCntrlHand;
        class KinectController {
        public:
            KinectController();
            virtual ~KinectController(); 

            //initialization -call 1 time
            int onInit();
            void assignKinectControllerToAllUIComponents();
            
            //init/timer/draw methods - override for each app in child class
            virtual void initWinVals(rtql8::renderer::RenderInterface* _mRI, int _wx, int _wy);

            virtual int initIKSkel(){return this->initIKSkel();}										//any initialization the skeleton from the file may need after being loaded, such as modifying initial position - put in child class
            virtual int initKinData(){return this->initKinData();}										//any initialization child class Kinect Controller might require - called from onInit
            virtual void initVars();																	//initialize KinController variables - replaced by xml

            virtual void onTimerQuery(){}																//get info for ui display - app specific

            virtual bool onKeyUpdate(unsigned char key, int x, int y);                                  //accept keyboard input - return false if no keyboard input detected/processed
            void onTimerUpdate();
            //drive kinect processing - customize for each application - no need to override, just set bool vals to include/exclude components
            void resetAllTimers();
            virtual void onDrawUpdate();																//display kinect results
            void draw2dUI();
            void drawIKUI();                                                                            //draw display-only UI objects in IK mode

            void loadKinSkelMarkerInfo();																//send current avatar joint info to kinect skeleton handler
            void StartKinIKSolver();																	//start solver to IK kin skel to avatar
            void StopKinIKSolver();																		//stop IK solver

            //getters/setters
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >* GetMarkers();					
            Eigen::Vector3d GetMarkerPos(int markerIndex);
            int GetMarkerCount();
            string GetKinSpeechRecog();
            vector<string> GetSkelJointVals();
            rtql8::kinematics::Skeleton* GetSkel(){return kinSkel;}
            bool getFlag(int idx){return this->KC_flags[idx];}

            void SetMarkerPos(int markerIndex, Eigen::Vector3d& pos);
            void setFlag(int idx, bool val);
            void SetShowModel(bool val);			
            void SetShowMarkers(bool val);		

            //setters for XML global variable population

            //kinHandler xml var setters
            bool setKinLibGlobalConstants(
                int tmp_RCD,                                              //_RCD              type:int        cmt:reconnect retry countdown timer
                long tmpkinMaxTilt,                                       //kinMaxTilt        type:long       cmt: max tilt angle
                long tmpkinMinTilt,                                       //kinMinTilt        type:long       cmt: min tilt angle
                wstring& tmpGrammarFileNameStr,                           //GramarFileNameStr type:string     cmt:file name for speech recognition grammar
                float tmpgripH,                                           //gripH             type:float      cmt: threshold plane for grip detection - height
                float tmpgripW,                                           //gripW             type:float      cmt: threshold plane for grip detection - width
                float tmpgripD,                                           //gripD             type:float      cmt: threshold plane for grip detection - depth
                int tmpLHandBx[4],                                        //LHandBx           type:int[4]     cmt:ara of vals for left hand detect box : {max x, min x-max x, max y, min y-max y}
                int tmpRHandBx[4],                                        //RHandBx           type:int[4]     cmt:ara of vals for right hand detect box : {max x, min x-max x, max y, min y-max y}
                NUI_TRANSFORM_SMOOTH_PARAMETERS tmpskelSmoothing,         //skelSmoothing     type:struct NUI_TRANSFORM_SMOOTH_PARAMETERS     cmt:used for kinect sdks internal calc of skeleton motion smoothing - see sdk for params
                float tmpHndTrkBxMult[6],                                 //HndTrkBxMult      type:float[6]   cmt:ara of multipliers for relative hand to root tracking box
                int tmpMX_CNT_FLR_Y,                                      //MX_CNT_FLR_Y      type:int        cmt:max number of consecutive frames of same floor depth counted (if kinects interpolated floor changes, decrement until 0 then change, to act as smoothing)
                float tmpSK_JNT_MULT[3],                                  //SK_JNT_MULT       type:float[3]   cmt:multiplier used when converting joint coords from skel space to display space
                float tmpSK_JNT_OFF[3],                                   //SK_JNT_OFF        type:float[3]   cmt:offset used when converting joint coords from skel space to display space
                double tmpjntSz[3],                                       //jntSz             type:double[3]  cmt:size of skel joint ara for display
                int tmpjntPosAraDpth,                                     //jntPosAraDpth     type:int        cmt:how many previous values for jnt pos,vel,accel we will keep around
                int tmpjntNumMaxInterp,                                   //jntNumMaxInterp   type:int        cmt:maximum interpolations before we just use most recent avatar value for joint ara location
                int tmpTRY_KIN_RCN,                                       //TRY_KIN_RCN       type:int        cmt:kinect controller reconnect retry countdown
                int tmpKC_hndEps_2D,                                      //KC_hndEps_2D      type:int        cmt:threshold amount hands need to move in xy to be detected as moving
                float tmpKinPshDist,                                      //KinPshDist        type:float      cmt: arbitrary, defines delta z to be considered push
                float tmpkinSkelDisp[3][3],                               //kinSkelDisp       type:float[3][3]        cmt:translation on screen for drawing raw kinect joint ara data noisy kin, filtered kin, avatar(postIK)
                float tmpKinIK_eps,                                       //KinIK_eps         type:float      cmt:IK eps amt
                float tmpKinIK_minPrtrb,                                  //KinIK_minPrtrb    type:float      cmt:minimum timestep amount we will allow for any single iteration
                int tmpKinIK_numIters,                                    //KinIK_numIters    type:int        cmt:number of IK iterations to run per timer cycle
                Eigen::VectorXd& tmpKinIK_weights,                        //KinIK_weights     type:Eigen::VectorXd    cmt:weights for joints in IK skel, idx'ed by kin joint ara idx - currently 20 joints in kin skel
                float tmpKinSldrBrdr,                                     //KinSldrBrdr       type:float      cmt:graphical border around slider object
                float tmpUIobj_DragSens,                                  //UIobj_DragSens    type:float      cmt:sensitivity of slider bar drag - 0.0-1.0
                float tmpUIobj_CrankSens,                                 //UIobj_CrankSens   type:float     cmt:sensitivity of crank bar drag - 0.0-1.0
                float tmpLHandDragSens,                                   //LHandDragSens     type:float    cmt:left hand sensitivity setting for when dragging
                float tmpRHandDragSens,                                   //RHandDragSens     type:float    cmt:right hand sensitivity setting for when dragging
                float tmpLHandPushSensIn,                                 //LHandPushSensIn   type:float      cmt:left hand push in sensitivity to detect click
                float tmpRHandPushSensIn,                                 //RHandPushSensIn   type:float      cmt:left hand push in sensitivity to detect click
                float tmpLHandPushSensOut,                                //LHandPushSensOut  type:float      cmt:right hand pull back sensitivity to disengage click
                float tmpRHandPushSensOut,                                //RHandPushSensOut  type:float      cmt:right hand pull back sensitivity to disengage click
                float tmpUIobj_MinFontScale,                              //type:float      cmt:min value for font scaling for ui object labels
                float tmpUIobj_MaxFontScale                               //type:float      cmt:max value for font scaling for ui object labels
            );

            bool setKinectHandlerFlag(int idx, bool vall);
            bool setKinAudioHandlerFlag(int idx, bool val);	
            bool setKinDepthHandlerFlag(int idx, bool val);	
            bool setKinImageHandlerFlag(int idx, bool val);	
            bool setKinInteractHandlerFlag(int idx, bool val);
            bool setKinSkelHandlerFlag(int idx, bool val);
            //set speech recognition strings and their subsequent state flag idx's
            void setKCspeechVals(string key, int val){              KCspeechVals[key] = val;        }

            bool setKinectControllerStateVars(int idx, bool val);	
            bool setKinectUIComponentFlag(int idx, bool val);
            				
            //draw various images
            void drawKinImage(bool isDepthImg, float trX, float trY, float trZ, float scX, float scY, float scZ);																				//draw either depth or rgb image from kinect
            void drawKinDpthHandImage(bool left, float trX, float trY, float trZ, float scX, float scY, float scZ);			//draw either left or right hand seg box from depth image
            void drawImage(GLubyte* imgData,GLuint imgTextureID, GLfloat imgW, GLfloat imgH, bool modelView);				//draws image - texture mapped to poly

            //drawing specific skeleton-related entities - can be overridden by child class
            virtual void drawKinSkelData(){}
            void drawKinSkelFrame(float trX,float trY, float trZ, int type, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs);
            void drawKinSkelClipBox(float trX, float trY, float trZ);
            void drawFullSkelBones(int status, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs);
            void drawFullSkelJoints(int status, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs);
            virtual void drawBone(int status, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs, int bnIdx);

            void buildNNet();

            //process and draw hands
            void setDpthHandState(bool disp, bool calc);
            void setUIMode(bool mode2D, bool mode3D, bool modeIK);

            void drawHands2d();                                   
            void drawHands3d();                                   

            void handleHands();

            virtual void setDispButtons(){}
            virtual bool checkUIElements(int& x, int& y, int& d, bool isMouse, int hand, int evnt);

            //turn on and off "screen paint" (2d ui)
            void enable2dUIMode(bool on);

            //build IK mode components - displayed but not interacted with during IK mode
            virtual void buildIKUI(vector<int>& numCmps);

            //build 2dUI components - particulars handled in MyKinController implementation
            virtual void build2dUI(vector<int>& numCmps);

                //the following default to 2d mode UI control - TODO : refactor to specify 2d in name
            void addUIButton(KinUIButton* btn);
            void addUIImageButton(KinUIImageButton* btn);
            int numUIButtons() { return Kin2dUIBtn.size(); }
            KinUIButton* getUIButton(int idx) { return Kin2dUIBtn[idx].get(); }
            int numUIImageButtons() { return Kin2dUIImgBtn.size(); }
            KinUIImageButton* getUIImageButton(int idx) { return Kin2dUIImgBtn[idx].get(); }
            KinUISlider* getUISlider(const char* const name);
            KinUICrank* KinectController::getUICrank(const char* const name);
            int numUITimers() { return Kin2dUITimer.size(); }
            KinUITimer* getUITimer(int idx) { return Kin2dUITimer[idx].get(); }
            void addPage(KinUIPage* page);
            KinUIPage* setPage(const char* const pagename);
            void updateAsPage(KinUIPage* page);
            bool deletePage(const char* const pagename);
            virtual bool parseSpeechCommand();
          
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
        protected :	//methods
            void audioUpdate();																	//process new frame of speech recognition data from kinect
            void clearSpeechRecogVals();														//clear speech recognition text once acted upon
            int loadIKSkel();																	//load avatar skeleton from file used for IK
            void runIKSolver();																	//solve IK for markers fromm kinect
            void updateMarkersAndIK();															//process new frame of skeleton data from kinect - get joint locations, perform IK
            //void loadKinSkelMarkerData();														//load kinect skeleton marker data from file, to use kinect when sensor not present

        public :    //vars
            int winX, winY;																		//window dimensions

            KinectHandler* getKinectHandler() { return kinHNDLR; }

        protected : //vars
            rtql8::kinematics::FileInfoSkel<rtql8::kinematics::Skeleton> skelModelFile;			//skeleton model file info
            rtql8::kinematics::Skeleton* kinSkel;												//skeleton

            const char* avatarSkelFileName;														//name of avatar skeleton file
            KinectHandler* kinHNDLR;															//controller for kinect
            KinSkelIKSolver* kinIKSolver;														//IK solver for IK from kin skel joints to avatar
            rtql8::renderer::RenderInterface* mRI;												//render interface used to display data, if one exists

            deque<bool> KC_flags;																//status flags for kinect controller
            deque<bool> UICmp_defFlags;                                                        //default flags for all ui components, from XML.  can be overridden for each component upon creation

            string kinSpeechRecog;																//recognized grammar word from speech recog
            int kinectRConnTimer;																//countdown to attempt reconnection if no kinect detected
            int mNumMarkers;																	//number of markers on skeleton to IK to
            //skel/ik structures
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> kinSkelMarkerPos;		//marker positions in mocap/kinect data
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> avatarIKHandles;		//locations of avatar markers after IK
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> kinSkelNoisyMarkers;	//locations of avatar markers after IK
            vector<int> kinSkelJointState;															//whether a joint is known, unknown or inferred

            vector<string> skelJointStrs;														//string values from kinect for each joint, used for display
            bool skelDrawn;																		//whether or not skeleton drawn to specific render interface

            vector<KinCntrlHand> KinHands;                                                      //user's hands from kinect streams                                                         

            GLuint LHandDpthImgID,RHandDpthImgID;												// ID of the texture to contain hand depth img for both hands - used only for debugging, not intended to be displayed

            Eigen::Vector3f oldMousePos;                                                        //last turn mouse position

            map<std::string, int> KCspeechVals;                                                 //map of speech recognition phrases to flag IDX's to set - received from xml config file

            vector<KinUIComponent*> manualUICmp;
            vector<std::shared_ptr<KinUIComponent> >    Kin2dUICmp;								//holding all specialised ui components intended to be displayed during 2D UI interaction mode
            vector<std::shared_ptr<KinUIComponent> >    KinIKUICmp;								//holding all specialised ui components intended to be displayed during IK mode

                //2d mode containers
            vector<std::shared_ptr<KinUIButton> >	    Kin2dUIBtn;								//holding kinect buttons
            vector<std::shared_ptr<KinUIImageButton> >	Kin2dUIImgBtn;							//holding kinect Image buttons
            vector<std::shared_ptr<KinUISlider> >	    Kin2dUISldr;							//holding kinect sliders
            vector<std::shared_ptr<KinUICrank> >		Kin2dUICrnk;							//holding kinect cranks
            vector<std::shared_ptr<KinUIPrgBar> >		Kin2dUIProgBar;							//holding kinect timers (specialized sliders that countdown) and progressbars (do the same thing, but progress upwards)
            vector<std::shared_ptr<KinUITimer> >		Kin2dUITimer;							//holding kinect timers (specialized sliders that countdown) and progressbars (do the same thing, but progress upwards)
            vector<std::shared_ptr<KinUITextBox> >		Kin2dUITextBox;							//holding kinect textboxes (modified buttons able to accept and process typed input)
            vector<std::shared_ptr<KinUICaption>>       Kin2dUICaption;
                //IK mode UI component containers
            vector<std::shared_ptr<KinUIButton> >	    KinIKUIBtn;								//holding kinect buttons
            vector<std::shared_ptr<KinUIPrgBar> >		KinIKUIProgBar;							//holding kinect timers (specialized sliders that countdown) and progressbars (do the same thing, but progress upwards)
            vector<std::shared_ptr<KinUITimer> >		KinIKUITimer;							//holding kinect timers (specialized sliders that countdown) and progressbars (do the same thing, but progress upwards)

                //UI pages
            vector<std::shared_ptr<KinUIPage> > KinUIPages;
            int currPage;                                                                       //currently displayed UI page

            KinNNet kNet;   
            KinUIPage* currentUIPage;
        };//KinectController class
    }//namespace kinect
}//namespace rtql8
#endif
