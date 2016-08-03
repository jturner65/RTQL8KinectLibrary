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

#include "KinHndlrConsts.h"
#include "KinectController.h"
#include <fstream>

namespace rtql8{
    namespace kinect{
        KinectController::KinectController(): 
            skelModelFile(), 
            avatarSkelFileName(KIN_IK_SKEL_NAME), 
            kinSpeechRecog(""), 
            kinectRConnTimer(TRY_KIN_RCN), 
            mNumMarkers(NUI_NumJnts), 
            KinHands(),
            kinSkelMarkerPos(NUI_NumJnts,Eigen::Vector3d(0, 0, 0)), 
            avatarIKHandles(NUI_NumJnts,Eigen::Vector3d(0, 0, 0)), 
            kinSkelNoisyMarkers(NUI_NumJnts,Eigen::Vector3d(0, 0, 0)), 
            kinSkelJointState(NUI_NumJnts),
            KC_flags(KC_numFlags, true), 
            UICmp_defFlags(UIobj_numFlags,false),
            skelJointStrs(NUI_NumJnts, ""),
            mRI(NULL),
            skelDrawn(false), currPage(0),
            Kin2dUICmp(), Kin2dUIBtn(),	Kin2dUIImgBtn(), Kin2dUISldr(),Kin2dUICrnk(), Kin2dUIProgBar(),
            KinIKUICmp(), KinIKUIBtn(), KinIKUIProgBar(), KCspeechVals()
        {
            currentUIPage = NULL;
            KinIK_weights = Eigen::VectorXd(NUI_NumJnts);
            KinIK_weights<<1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;         //initialize with 1's in case xml doesn't init properly
	        //populate constants and state vars from default values
            clog<<"KinectController : Read default xml file"<<std::endl;
            rtql8::kinect::KinCntrlrXMLReader config(this, false);          //set auto upd to false, so that flags are updated after kinect handler is instantiated.
            config.readFile(RTQL8_DATA_PATH"kinectUIDefault.xml");
            clog<<"KinectController : Done reading default xml file"<<std::endl;

            kinHNDLR = new KinectHandler();
            clog<<"KinectController : Kinect sensor connected (KC CNSTRCTR) : "<<((kinHNDLR->kinConnected()) ? "true" : "false")<<std::endl;
            kinIKSolver = new KinSkelIKSolver();
            clog<<"KinectController : Kinect IK Solver initialized"<<std::endl;
            config.updateHandlerStateFlags();                               //update initial state of flags upon instantiation of handlers 
        }//cnstrctr

        KinectController::~KinectController(){
            delete kinIKSolver;
            delete kinHNDLR;
            delete kinSkel;
            delete avatarSkelFileName;
        }//dstrctr

        void KinectController::initWinVals(rtql8::renderer::RenderInterface* _mRI, int _wx, int _wy){
                clog<<"base class init win vals"<<endl;
                mRI = _mRI;
                winX = _wx; 
                winY = _wy;
                    //set initial hand locations and initialize hands
                KinHands[KC_LEFT].setInitLoc(2*winX/5, winY/2);
                KinHands[KC_RIGHT].setInitLoc(3*winX/5, winY/2);
        }//initWinVals

        //called once, to initialize IK avatar skeleton and send initial av skeleton marker info to kinect sensor handler
        int KinectController::onInit(){
            initVars();
            int retVal = loadIKSkel();
            if (0 != retVal) {	clog<<"KinectController : Error Loading Kin IK Skel File : "<<string(avatarSkelFileName)<<std::endl;		return retVal;	}
            retVal = this->initIKSkel();						//call instantiating class's version
            if (0 != retVal) {	clog<<"KinectController : Error Initializing Kin IK Skel"<<std::endl;									return retVal;	}
            //retVal = this->initKinData();						//call instancing class's version - init any other kinect data - replaced by xml-loaded data
            //if (0 != retVal) {	clog<<"KinectController : Error Initializing Kinect Controller data "<<std::endl;						return retVal;	}
            if((!kinHNDLR->kinValid()) || (!kinHNDLR->kinSkelHandlerAvail())){ 
                if(!kinHNDLR->kinValid()){	clog<<"KinectController : No Valid KinHandler available "<<std::endl;						return 1;		}
                else if(!kinHNDLR->kinSkelHandlerAvail()){	clog<<"KinectController : No Valid KinSkelHandler available "<<std::endl;	return 2;		}
            }	//send most recent IK joint pos to kin	
            loadKinSkelMarkerInfo();	
            kinIKSolver->buildWeights(KinIK_weights);           //update weights

            return 0;											//completed initialization acceptably
        }//onInit

        void KinectController::assignKinectControllerToAllUIComponents() {
            for (int i = 0; i < KinUIPages.size(); i++) {
                KinUIPage* page = KinUIPages[i].get();
                for (int j = 0; j < page->KinPgUICmp.size(); j++) {
                    KinUIComponent* cmp = page->KinPgUICmp[j].get();
                    cmp->kinectController = this;
                    // cout << "assign!!" << endl;
                }
            }

            for (int j = 0; j < this->Kin2dUICmp.size(); j++) {
                KinUIComponent* cmp = this->Kin2dUICmp[j].get();
                cmp->kinectController = this;
                // cout << "assign!!" << endl;
            }
            for (int j = 0; j < this->KinIKUICmp.size(); j++) {
                KinUIComponent* cmp = this->KinIKUICmp[j].get();
                cmp->kinectController = this;
                // cout << "assign!!" << endl;
            }

        }

        void KinectController::initVars(){
            clog<<"base class initVars"<<endl;
            //following needs to be called after kinectCTL is attempted to be initialized - 1st is display hand boxes, 2nd is calculate grip
            setDpthHandState(true, true);
            setUIMode(true, false, false);				
            //setUIMode(false, false, true);				
            for(int hnd = 0; hnd < 2; ++hnd){           //left hand = 0, right hand = 1 
                KinCntrlHand newHand(this, hnd);
                KinHands.push_back(newHand);
            }
            oldMousePos = Eigen::Vector3f(-1,-1,-1);
        }//initVars

        //load avatar skeleton model
        int KinectController::loadIKSkel(){
            clog<<"\tKinectController : about to load Kin IK Skel File : "<<string(avatarSkelFileName)<<std::endl;

            bool readSkel = skelModelFile.loadFile(avatarSkelFileName, SKEL);								//SKEL defined in enum in rtql8::kinematics::SkeletonFileType
            if(false == readSkel){	clog<<"\tKinectController : Kinect Avatar Skeleton file : "<<string(avatarSkelFileName)<<" does not exist" << endl; return 12;	}	
            clog<<"\tKinectController : Loaded Kin IK Skel File : "<<string(avatarSkelFileName)<<std::endl;

            kinSkel = skelModelFile.getSkel();
            clog<<"\tKinectController : about to init solver with avatar skeleton"<<std::endl;
            kinIKSolver->initSolver(kinSkel);											

            clog<<"\tKinectController : number of dofs in Kinect Avatar skeleton: " << kinSkel->getNumDofs()<< endl;
            if(kinIKSolver->isReadyToSolve()){	kinIKSolver->runSolver = true;}
            return 0;
        }//loadIKSkel

        //all timer functionality - enable/disable via boolean flags
        void KinectController::onTimerUpdate(){
            if(kinHNDLR->kinValid()) {
                kinectRConnTimer = TRY_KIN_RCN;
                //if(KC_flags[_KC_USE_STRM_INTR]){		kinHNDLR->buildInteractData();	}//if we want to process interactions (hand grabbing - currently using depth image and our own code to determine hand grab)	
                if(KC_flags[_KC_USE_STRM_SKEL]){																//if we want IK on kinect skeleton
                    //if(kinHNDLR->kinValidIKSkelAvail()){		
                    updateMarkersAndIK();	
                    //}		//get markers from kin data for ik, and perform IK if valid data available		
                }//_KC_USE_STRM_SKEL
                //if(KC_flags[_KC_USE_STRM_AUD]){	if(kinHNDLR->kinAudioHandlerAvail()){	audioUpdate();}	}	//if we are using speech recognition
                if(kinHNDLR->getFlag(_KH_SKEL_CALC_HND)){				handleHands();			}				    //get current state of hands from skel handler and process for UI events
            } else if(kinectRConnTimer <= 0){
                kinHNDLR->checkKinectConn();	kinectRConnTimer = TRY_KIN_RCN;
            } else { //if kin valid else
                --kinectRConnTimer;
            }//if kin valid else
            
            if (KC_flags[_KC_DISP_UI_2D]) {             //for countdown timers
                for(auto tmr = Kin2dUITimer.begin(); tmr != Kin2dUITimer.end(); ++tmr){
                    if((*tmr)->isCntDnTimer()){           (*tmr)->advTimerVal();    }
                }
            } else if(KC_flags[_KC_DISP_UI_IK]){
                for(auto tmr = KinIKUITimer.begin(); tmr != KinIKUITimer.end(); ++tmr){
                    if((*tmr)->isCntDnTimer()){
                        (*tmr)->advTimerVal();
                    }
                }
            }
        }//onTimerUpdate


        void KinectController::resetAllTimers() {
            // cout << "resetAllTimers" << endl;
            for(auto tmr = Kin2dUITimer.begin(); tmr != Kin2dUITimer.end(); ++tmr){
                (*tmr)->reset();
                // cout << "reset!!!!" << endl;
            }
        }//resetAllTimers

        //accept keyboard input - return false if no keyboard input detected/processed
        //move through all input-accepting objects, see if any are accepting input, if any are, return true and process input, if none, return false
        bool KinectController::onKeyUpdate(unsigned char key, int x, int y){
            bool retVal = false;
            for(auto pObj = Kin2dUITextBox.begin(); pObj != Kin2dUITextBox.end(); ++pObj){
                if((*pObj)->isBeingEdited()){
                    (*pObj)->acceptInput(key);              //returns whether still being edited after this cycle (i.e. is false if user entered "newline" or "esc"
                    retVal = true;
                    //return true;                          //if no need to go through all editable objects, return here
                }
            }           
            return retVal;
        }//onKeyUpdate

        //drawing routine called from child class
        void KinectController::onDrawUpdate(){	
            // cout << "draw2d: " << KC_flags[_KC_DISP_UI_2D] << endl;
            if(KC_flags[_KC_USE_STRM_SKEL]){	
                kinHNDLR->buildSkeleton();	
                updateMarkersAndIK();																		
            }								//if we want IK on kinect skeleton
            if(KC_flags[_KC_USE_STRM_AUD]){	if(kinHNDLR->kinAudioHandlerAvail()){	audioUpdate();}	}		//if we are using speech recognition
            //if(KC_flags[_KC_USE_STRM_INTR]){		kinHNDLR->buildInteractData();	}						//if we want to process interactions (hand grabbing - currently using our own code to determine hand grab)	
            //if(kinHNDLR->getFlag(_KH_SKEL_CALC_HND)){				handleHands();			}				//moved to timer
					
            //if(KC_flags[_KC_USE_STRM_DPTH]){}																//if we want to process depth image 
            //if(KC_flags[_KC_USE_STRM_RGB]){}																//if we want to process rgb image 
            if (KC_flags[_KC_DISP_UI_2D]) {
                draw2dUI();
            } else if(KC_flags[_KC_DISP_UI_IK]){
                drawIKUI();
            }
               //draw 2d UI if one exists and currently displaying it
            //if (KC_flags[_KC_DISP_UI_3D]){}
            //if (KC_flags[_KC_DISP_UI_IK]){drawIKUI();}
            //if (KC_flags[_KC_PROC_HANDS_3D]){drawHands3d();}												//draw hands in 3d space
        }//onDrawUpdate

        ////////
        // draw various kinect images and data
        ////////

        //draw displayable components of 2D ui on screen
        void KinectController::draw2dUI(){
            glPushMatrix();
            enable2dUIMode(true);
            for(int i = 0; i < Kin2dUICmp.size(); ++i){
                glPushMatrix();
                Kin2dUICmp[i]->KinUIComponent::draw();
                glPopMatrix();
            }
            if(kinHNDLR->kinValid() && KC_flags[_KC_DISP_UI_IK] == false) {
                drawHands2d();
            }
            enable2dUIMode(false);
            glPopMatrix();
        }//		

        //draw displayable UI components during IK mode
        void KinectController::drawIKUI(){
            glPushMatrix();
            enable2dUIMode(true);
            for(int i = 0; i < KinIKUICmp.size(); ++i){
                glPushMatrix();
                KinIKUICmp[i]->KinUIComponent::draw();
                glPopMatrix();
            }
            enable2dUIMode(false);
            glPopMatrix();
        }//
		
        //toggle 2d ui mode or 3d object mode in drawing
        void KinectController::enable2dUIMode(bool on){
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();		
            if(on){
                glDisable(GL_LIGHTING | GL_DEPTH_TEST);
                glDepthMask(0);
                glOrtho(0, winX, winY,0,-1,1);
            } else {
                glEnable(GL_DEPTH_TEST | GL_LIGHTING);
                glDepthMask(1);
                gluPerspective(45.0f, (double)winX/(double)winY, 1.0, 100.0);
            }
            glViewport(0,0,winX,winY);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
        }//set2dUIMode

        void KinectController::drawHands2d(){
            glPushMatrix();
                glDisable(GL_LIGHTING);
                this->KinHands[KC_LEFT].draw2d();
                this->KinHands[KC_RIGHT].draw2d();
                glEnable(GL_LIGHTING);
            glPopMatrix();
        }//drawHands2d

        //draw hands in 3d space
        void KinectController::drawHands3d(){
            glPushMatrix();
                glDisable(GL_LIGHTING);
                this->KinHands[KC_LEFT].draw3d();
                this->KinHands[KC_RIGHT].draw3d();
                glEnable(GL_LIGHTING);
            glPopMatrix();
        }//drawHands2d

        //true for depth image, false for RGB image - image from rgb or depth streams from kinect
        void KinectController::drawKinImage(bool isDepthImg, float trX, float trY, float trZ, float scX, float scY, float scZ){
            glPolygonMode(GL_FRONT_AND_BACK,  GL_FILL);								//fills polys,  necessary for images to display
            glPushMatrix();
            GLubyte* imgData = kinHNDLR->buildImage(isDepthImg);
            GLuint imgTextureID = kinHNDLR->getImageTexID(isDepthImg);
            if((NULL != imgData) && (-1 != imgTextureID)) { 
                glTranslatef(trX,trY,trZ);
                if((1!=scX) || (1!=scY) || (1!=scZ)){		glScalef(scX,scY,scZ);}
                drawImage(imgData, imgTextureID, strmImgW, strmImgH, true);
            }
            glPopMatrix();
        }//drawKinImage

        //true for left hand, false for right hand - image of hand isolated from depth image
        void KinectController::drawKinDpthHandImage( bool left, float trX, float trY, float trZ, float scX, float scY, float scZ){
            glPolygonMode(GL_FRONT_AND_BACK,  GL_FILL);								//fills polys,  necessary for images to display
            glPushMatrix();
            GLubyte* imgData = kinHNDLR->getDepthHandImg(left);
            int width = kinHNDLR->getHandImageWidth(left), height = kinHNDLR->getHandImageHeight(left);
            if((NULL != imgData) && (-1 != (left ? this->LHandDpthImgID : this->RHandDpthImgID))) { 
                glTranslatef(trX,trY,trZ);
                if((1!=scX) || (1!=scY) || (1!=scZ)){		glScalef(scX,scY,scZ);}
                drawImage(imgData, (left ? this->LHandDpthImgID : this->RHandDpthImgID), width, height, true);
            }
            glPopMatrix();
        }//drawKinImage

        //draws a 2d image in 3d space
        void KinectController::drawImage(GLubyte* imgData,GLuint imgTextureID, GLfloat imgW, GLfloat imgH, bool modelView){
            glPushMatrix();
                //if(modelView){							enable2dUIMode(true);}
                glColor4d(0.8,0.8,0.8,1.0);
                glScalef(.01f,.01f,1.0f);	
                glTranslatef(-imgW/2.0f,0,0);				//center in x on screen
                glEnable(GL_TEXTURE_2D);
                glDisable(GL_LIGHTING);
                glBindTexture(GL_TEXTURE_2D, imgTextureID);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);    
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);    
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imgW, imgH, 0, GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid*)imgData);	

                glBegin( GL_QUADS);
                    glTexCoord2f(0, 1);      glVertex3f(0, 0, 0);				
                    glTexCoord2f(1, 1);      glVertex3f(imgW, 0, 0);				
                    glTexCoord2f(1, 0);      glVertex3f(imgW, imgH, 0.0f);		
                    glTexCoord2f(0, 0);      glVertex3f(0, imgH, 0.0f);			
                glEnd();
                glDisable(GL_TEXTURE_2D);
                glEnable(GL_LIGHTING);
                //if(modelView){							enable2dUIMode(false);}
            glPopMatrix();
        }//drawKinImage
		
        void KinectController::drawKinSkelFrame(float trX,float trY, float trZ, int type, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs){
            glPushMatrix();
                glTranslatef(trX,trY,trZ);
                drawFullSkelBones(type, jointLocs);
                drawFullSkelJoints(type, jointLocs);
            glPopMatrix();
        }//drawKinSkelFrame

        //draw clipping frame around kinect joint tracking
        void KinectController::drawKinSkelClipBox(float trX, float trY, float trZ){
            //min/max x : idx 0,1 min/max y : idx 2,3  min/max z	: idx 4,5
            vector<double> box = vector<double>( kinHNDLR->getClippingBox());
            glPushMatrix();
                glTranslatef(trX,trY,trZ);
                glColor4f(1,0,0,.3f);
                glBegin(GL_QUADS);									//floor																	//floor from skel frame data
                glNormal3d(0,1,0);																										//FloorCrnrsY idx : 0 minXmaxZ, 1 minXminZ, 2 maxXminZ, 3 maxXmaxZ
                glVertex3d(box[0], box[2], box[5]);													//min x min y max z
                glVertex3d(box[0], box[2], box[4]);													//min x min y min z
                glVertex3d(box[1], box[2], box[4]);													//max x min y min z
                glVertex3d(box[1], box[2], box[5]);													//max x min y max z
                glEnd();
						
                glBegin( GL_QUAD_STRIP );
                    //right wall
                    glVertex3d(box[1], box[2], box[5]);													//max x min y max z
                    glVertex3d(box[1], box[2], box[4]);													//max x min y min z
                    //ceiling
                    glVertex3d(box[1], box[3], box[5]);													//max x max y max z							
                    glVertex3d(box[1], box[3], box[4]);													//max x max y min z
                    //left wall					 	  
                    glVertex3d(box[0], box[3], box[5]);													//min x max y max z
                    glVertex3d(box[0], box[3], box[4]);													//min x max y min z
                    //final    
                    glVertex3d(box[0], box[2], box[5]);													//min x min y max z
                    glVertex3d(box[0], box[2], box[4]);													//min x min y min z
                glEnd();
            glPopMatrix();	
        }//drawKinSkelClipBox

        //draw all bones for passed joint data
        void KinectController::drawFullSkelBones(int status, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs){
            glPushMatrix();
            for(int bnIdx = 0; bnIdx < NUI_NumBones; ++bnIdx){			drawBone(status, jointLocs, bnIdx);}
            glPopMatrix();
        }//drawFullSkelBones

        void KinectController::drawFullSkelJoints(int status, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs){
            ShapeEllipsoid temp(Eigen::Vector3d(jntSz[0], jntSz[1], jntSz[2]), 0.0);									
            Eigen::Vector3d clr(jntClr[status][0],jntClr[status][1],jntClr[status][2]);												
            Eigen::Vector3d clrInf(jntClr[infClrIDX][0],jntClr[infClrIDX][1],jntClr[infClrIDX][2]);

            for (int jIdx = 0; jIdx < jointLocs.size(); ++jIdx){
                int jointState = (SK_AVTR == status ? 2 : kinSkelJointState[jIdx]);
                if((SK_AVTR != status) && (NUI_SKELETON_POSITION_NOT_TRACKED == jointState)){	continue;}							//if not avatar and joint is not tracked, don't attempt to draw this joint
                if((SK_AVTR != status) && (NUI_SKELETON_POSITION_INFERRED == jointState)){		temp.setColor(clrInf);	}
                else{																			temp.setColor(clr);		}

                glPushMatrix();				
                    glTranslated(jointLocs[jIdx](0), jointLocs[jIdx](1), jointLocs[jIdx](2));
                    temp.draw(mRI);																//if render interface not defined/null, will not draw sphere
                glPopMatrix();
            }//for
        }//drawFullSkelJoints

        void KinectController::drawBone(int status, const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> &jointLocs, int bnIdx){
            int joint0 = bnJoints[bnIdx][0], joint1 = bnJoints[bnIdx][1];															//find joint idx for each end of bone
            int joint0State = (SK_AVTR == status ? 2 : kinSkelJointState[joint0]);
            int joint1State = (SK_AVTR == status ? 2 : kinSkelJointState[joint1]);
            if((SK_AVTR != status) &&  ((NUI_SKELETON_POSITION_NOT_TRACKED == joint0State) && (NUI_SKELETON_POSITION_NOT_TRACKED == joint1State))){	return;}									//if not avatar and neither joint is tracked, don't draw		
		
            if ((NUI_SKELETON_POSITION_INFERRED == joint0State) || (NUI_SKELETON_POSITION_INFERRED == joint1State)){					
                glColor4f( boneClr[infClrIDX][0], boneClr[infClrIDX][1], boneClr[infClrIDX][2], boneClr[infClrIDX][3] );
            } else {																
                glColor4f( boneClr[status][0], boneClr[status][1], boneClr[status][2], boneClr[status][3] );
            }
            glBegin( GL_LINES );
                glVertex3f(jointLocs[joint0][0] , jointLocs[joint0][1] ,jointLocs[joint0][2]);
                glVertex3f(jointLocs[joint1][0] , jointLocs[joint1][1] ,jointLocs[joint1][2]);
            glEnd();
        }//drawBone

        //whether or not we are tracking hand state in dpth image - we will want to turn off IK and freeze motion when we are doing this or performance may suffer
        void KinectController::setDpthHandState(bool disp, bool calc){
            kinHNDLR->setFlag( _KH_DPTH_DISP_HND, disp);
            kinHNDLR->setFlag( _KH_SKEL_CALC_HND, calc);
        }//setHandState

        //whether or not we are processing the UI via the hand icons - eventually turn off UI when doing this
        void KinectController::setUIMode(bool mode2D, bool mode3D, bool modeIK){
            this->KC_flags[_KC_PROC_HANDS_2D] = mode2D;
            this->KC_flags[_KC_PROC_HANDS_3D] = mode3D;
            this->KC_flags[_KC_PROC_IK] = modeIK;			        
        }//


        ////////
        // UI building and event handling
        ////////

        void KinectController::handleHands(){			                    //currently 2d only
            Eigen::VectorXd handValLocAra = kinHNDLR->getSkelRawHandPos();                              //idx's 0,1,2 for left hand, 3,4,5 for right, 6,7,8 for root, 9,10,11 for r shoulder
            int lGripHandSt = -1, rGripHandSt = -1;
            if (this->KC_flags[_KC_PROC_2D_DGRAB]){                                                 //if processing grab in 2d space
                int getPressGripState = kinHNDLR->getPressGripState();
                lGripHandSt = (0 == getPressGripState % 10) ? 0 : 2;								
                rGripHandSt = (0 == getPressGripState / 10) ? 0 : 2;
            }
            //call both hand objs individually
            // rebuild array so that only has 1 hand's worth of data, along with root and shoulder
            Eigen::VectorXd lHandValLocAra(9), rHandValLocAra(9);
            lHandValLocAra<<handValLocAra(0),handValLocAra(1),handValLocAra(2),handValLocAra(6),handValLocAra(7),handValLocAra(8),handValLocAra(9),handValLocAra(10),handValLocAra(11);
            rHandValLocAra<<handValLocAra(3),handValLocAra(4),handValLocAra(5),handValLocAra(6),handValLocAra(7),handValLocAra(8),handValLocAra(9),handValLocAra(10),handValLocAra(11);
                //call hand routines
            KinHands[KC_LEFT].handleHand(lHandValLocAra, lGripHandSt);
            KinHands[KC_RIGHT].handleHand(rHandValLocAra, rGripHandSt);

            this->setDispButtons();

        }//handleHands

        //deprecated : now using XML loading of objects
        //instantiate various kinect gui components 
        //numcpms is number of each type of component, by idx
        //0:buttons 1:sliders 2:cranks 3:image buttons 4:timers
        //supplanted by XML, this is only for demo
        void KinectController::build2dUI(vector<int>& numCmps){
            int chckVal = 0;
            if(numCmps.size() > chckVal){
                for(int i = 0; i < numCmps[chckVal]; ++i){			//buttons
                    std::shared_ptr<KinUIButton> tmpBut (new KinUIButton(0,0));
                    std::shared_ptr<KinUIComponent> tmpBsBut = tmpBut;
                    Kin2dUIBtn.push_back(tmpBut);
                    Kin2dUICmp.push_back(tmpBsBut);
                }
                chckVal++;
            if(numCmps.size() > chckVal){
                for(int i = 0; i < numCmps[chckVal]; ++i){			//sliders
                    std::shared_ptr<KinUISlider> tmpSld (new KinUISlider(0,0));
                    std::shared_ptr<KinUIComponent> tmpBsSld = tmpSld;
                    Kin2dUISldr.push_back(tmpSld);
                    Kin2dUICmp.push_back(tmpBsSld);
                }			
                chckVal++;
            if(numCmps.size() > chckVal){	
			    for(int i = 0; i < numCmps[chckVal]; ++i){			//cranks
                    std::shared_ptr<KinUICrank> tmpCrnk (new KinUICrank(0,0));
                    std::shared_ptr<KinUIComponent> tmpBsSld = tmpCrnk;
                    Kin2dUICrnk.push_back(tmpCrnk);
                    Kin2dUICmp.push_back(tmpBsSld);
			    }	
                chckVal++;
			if(numCmps.size() > chckVal){	
                for(int i = 0; i < numCmps[chckVal]; ++i){			//image buttons
				    std::shared_ptr<KinUIImageButton> tmpImgBut (new KinUIImageButton(0,0));
				    std::shared_ptr<KinUIComponent> tmpBsSld = tmpImgBut;
				    Kin2dUIImgBtn.push_back(tmpImgBut);
				    Kin2dUICmp.push_back(tmpBsSld);
                }			
                chckVal++;
			if(numCmps.size() > chckVal){	
                for(int i = 0; i < numCmps[chckVal]; ++i){			//progbar (sliders that display progress and don't accept input)
                    std::shared_ptr<KinUIPrgBar> tmpPrgBar (new KinUIPrgBar(0));
				    std::shared_ptr<KinUIComponent> tmpBsSld = tmpPrgBar;
				    Kin2dUIProgBar.push_back(tmpPrgBar);
				    Kin2dUICmp.push_back(tmpBsSld);
                }			
                chckVal++;
			if(numCmps.size() > chckVal){	
                for(int i = 0; i < numCmps[chckVal]; ++i){			//timers (sliders that countdown and don't accept input)
                    std::shared_ptr<KinUITimer> tmpTimer (new KinUITimer(0));
				    std::shared_ptr<KinUIComponent> tmpBsSld = tmpTimer;
				    Kin2dUITimer.push_back(tmpTimer);
				    Kin2dUICmp.push_back(tmpBsSld);
                }			
                chckVal++;
			if(numCmps.size() > chckVal){	
                for(int i = 0; i < numCmps[chckVal]; ++i){			//textboxes (buttons that accept keyboard input)
                    std::shared_ptr<KinUITextBox> tmpTB (new KinUITextBox(0,0));
				    std::shared_ptr<KinUIComponent> tmpBsSld = tmpTB;
				    Kin2dUITextBox.push_back(tmpTB);
				    Kin2dUICmp.push_back(tmpBsSld);
                }			
                chckVal++;
			if(numCmps.size() > chckVal){	
                for(int i = 0; i < numCmps[chckVal]; ++i){			//caption (buttons that accept keyboard input)
                    std::shared_ptr<KinUICaption> tmpCap (new KinUICaption(0,0));
				    std::shared_ptr<KinUIComponent> tmpBsSld = tmpCap;
				    Kin2dUICaption.push_back(tmpCap);
				    Kin2dUICmp.push_back(tmpBsSld);
                }			
                chckVal++;
            }}}}}}}}//checking how many values are in numCmps 
        }//build2dUI

        //instantiate various kinect gui components used during IK mode (display only)
        //numcpms is number of each type of component, by idx
        //0:buttons 1:timers - supplanted by XML, only for demo
        void KinectController::buildIKUI(vector<int>& numCmps){
            int chckVal = 0;
            if(numCmps.size() > chckVal){
                for(int i = 0; i < numCmps[chckVal]; ++i){			//buttons
                    std::shared_ptr<KinUIButton> tmpBut (new KinUIButton(0,0));
                    std::shared_ptr<KinUIComponent> tmpBsBut = tmpBut;
                    KinIKUIBtn.push_back(tmpBut);
                    KinIKUICmp.push_back(tmpBsBut);
                }
                chckVal++;
			if(numCmps.size() > chckVal){	
                for(int i = 0; i < numCmps[chckVal]; ++i){			//timers (sliders that countdown and don't accept input)
                    std::shared_ptr<KinUIPrgBar> tmpPrgBar (new KinUIPrgBar(0));
				    std::shared_ptr<KinUIComponent> tmpBsPBr = tmpPrgBar;
				    KinIKUIProgBar.push_back(tmpPrgBar);
				    KinIKUICmp.push_back(tmpBsPBr);
                }			
                chckVal++;
			if(numCmps.size() > chckVal){	
                for(int i = 0; i < numCmps[chckVal]; ++i){			//timers (sliders that countdown and don't accept input)
                    std::shared_ptr<KinUITimer> tmpTimer (new KinUITimer(0));
				    std::shared_ptr<KinUIComponent> tmpBsTmr = tmpTimer;
				    KinIKUITimer.push_back(tmpTimer);
				    KinIKUICmp.push_back(tmpBsTmr);
                }			
                chckVal++;
            }}}//checking how many values are in numCmps 
        }//buildIKUI

        //add a button to the UI 
        void KinectController::addUIButton(KinUIButton* btn) {
            //for (int j = 0; j < Kin2dUIBtn.size(); j++) {
            //    KinUIComponent* cmp = Kin2dUIBtn[j].get();
            //    if (cmp == btn) {
            //        cout << "REDUNDANT!!!!!!!!!" << endl;
            //        return;
            //    }
            //}


            std::shared_ptr<KinUIButton>    tmp0 (btn);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            Kin2dUIBtn.push_back(tmp0);
            Kin2dUICmp.push_back(tmp1);
        }

        void KinectController::addUIImageButton(KinUIImageButton* btn) {
            //for (int j = 0; j < Kin2dUIImgBtn.size(); j++) {
            //    KinUIComponent* cmp = Kin2dUIImgBtn[j].get();
            //    if (cmp == btn) {
            //        cout << "REDUNDANT!!!!!!!!!" << endl;
            //        return;
            //    }
            //}

            std::shared_ptr<KinUIImageButton>    tmp0 (btn);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            Kin2dUIImgBtn.push_back(tmp0);
            Kin2dUICmp.push_back(tmp1);
        }

        KinUISlider* KinectController::getUISlider(const char* const name) {
            std::string strname(name);
            for (int i = 0; i < Kin2dUISldr.size(); i++) {
                KinUISlider* sld = Kin2dUISldr[i].get();
                std::string label = sld->getLabel();
                if (label.length() < strname.length()) {              continue;             }
                std::string prefix = label.substr(0, strname.length());
                if (prefix == strname) {              return sld;            }
            }
            return NULL;
        }

        KinUICrank* KinectController::getUICrank(const char* const name) {
            std::string strname(name);
            for (int i = 0; i < Kin2dUICrnk.size(); i++) {
                KinUICrank* cnk = Kin2dUICrnk[i].get();
                std::string label = cnk->getLabel();
                if (label.length() < strname.length()) {              continue;             }
                std::string prefix = label.substr(0, strname.length());
                if (prefix == strname) {              return cnk;            }
            }
            return NULL;
        }


        void KinectController::addPage(KinUIPage* page) {
            std::shared_ptr<KinUIPage>    tmp (page);
            KinUIPages.push_back(tmp);
        }

        KinUIPage* KinectController::setPage(const char* const pagename) {
            std::string name(pagename);
            for (int i = 0; i < KinUIPages.size(); i++) {
                KinUIPage* page = KinUIPages[i].get();
                if (name == page->name) {
                    updateAsPage(page);
                    return page;
                }
            }
            return NULL;
        }//setPage

        //set passed page as current UI components to be processed
        void KinectController::updateAsPage(KinUIPage* page) {
            this->currentUIPage = page;
            this->Kin2dUIBtn = page->KinPgUIBtn;
            this->Kin2dUICmp = page->KinPgUICmp;
            this->Kin2dUISldr = page->KinPgUISldr;
            this->Kin2dUICrnk = page->KinPgUICrnk;
            this->Kin2dUIImgBtn = page->KinPgUIImgBtn;
            this->Kin2dUIProgBar = page->KinPgUIProgBar;
            this->Kin2dUITimer = page->KinPgUITimer;
            this->Kin2dUITextBox = page->KinPgUITextBox;
            this->Kin2dUICaption = page->KinPgUICaption;
            resetAllTimers();
        }

        bool KinectController::deletePage(const char* const pagename) {
            std::string name(pagename);
            vector<std::shared_ptr<KinUIPage> > new_pages;

            bool found = false;
            for (int i = 0; i < KinUIPages.size(); i++) {
                KinUIPage* page = KinUIPages[i].get();
                if (name == page->name) {
                    found = true;
                } else {
                    new_pages.push_back(KinUIPages[i]);
                }
            }
            KinUIPages = new_pages;
            return found;
        }

        //UIpage should own the primary functionality here!
        //go through all elements and check to see if events have occured in them
        //d < 0 means event triggered by mouse
        //mouseEvent : if this is an event generated by the mouse or by the kinect
        //hand : which hand/button was cliked
        //evnt : the nature of the event : 0 : up, 1 : click/push, 2 : drag/grab
        bool KinectController::checkUIElements(int& x, int& y, int& d, bool mouseEvent, int hand, int evnt){
            if(!KC_flags[_KC_DISP_UI_2D]) {return false;}                                                               //only process events if in ui display mode
            bool objFound = false;
            if(mouseEvent){                this->setFlag(_KC_MOUSE_USE, true);}
            //migrating to UI page
            //bool objFound = this->KinUIPages[this->currPage]->checkUIElements(x,y,d,mouseEvent,hand,evnt);
            //oldMousePos = Eigen::Vector3f(x,y,0);

            if(mouseEvent) { 
                //mouse events : mouse action has triggered event
                bool msDragEvent = (2 == evnt) ? true : false;															//drag
                bool msClickEvent = (1 == evnt) ? true : false;												            //handle grip differently - 0:click up, 1:click down
                int m = Kin2dUICmp.size();
                int n = Kin2dUICmp.size() + manualUICmp.size();
                for(int idx = 0; idx < n; ++idx){
                    KinUIComponent* cmp = NULL;
                    if (idx < m) {
                        cmp = Kin2dUICmp[idx].get();
                    } else {
                        cmp = manualUICmp[idx - m];
                    }
                    
                    int clkVal = (cmp->isInside(x,y));                                                      //value returned from isInside - what part of object click was inside
                    if((0 != clkVal) || (cmp->checkDragByThisCntrl(KC_MOUSE))){                             //check if inside this object, or if this object is currently being dragged
                        if(!msDragEvent) {
                            if(msClickEvent){	
                                cmp->setClicked(x,y,clkVal, KC_MOUSE);
                            } else {   cmp->clearClicked(x,y, KC_MOUSE);}                                   //upon click up clear click event - needs to happen cycle after "click release" has been processed
                        } else {
                            int resD = cmp->setDragged(msDragEvent,x,y, KC_MOUSE);
                            if (resD != -1){
                                cmp->stickToClick(x,y);
                            }
                        }										                                                                //set drag state to be whether or not this is a drag event						
                        objFound = true;	
                    } else { cmp->clearClicked(x,y, KC_MOUSE);}									                    //use location = -1,-1 for click release of object with mouse outside of object (location ignored)
                }
                oldMousePos = Eigen::Vector3f(x,y,0);
            } else {																									        //hand ui event
                //bool hndDragEvent = (2 == evnt)? true : false;                                                                //tmp change to hand handling - push acts as drag too               
                //bool hndDragEvent = (1 == evnt) || (2 == evnt)? true : false;                        
                int oldHandState = this->KinHands[hand].getOldHandState();
                bool hndDragEvent = (2 == evnt) || this->KinHands[hand].getFlag(_KHnd_HAND_DRAG_OBJ);                        
                bool hndClickEvent = (1 == evnt) || (2 == evnt) ? true : false;											        //need to treat a grab like a click and a grab
                if(this->KinHands[hand].chkStClkUp()){this->KinHands[hand].setFlags(_KHnd_HAND_CLCKD,false);}                   //not in "clicked" state if transition from click to up
                int m = Kin2dUICmp.size();
                int n = Kin2dUICmp.size() + manualUICmp.size();
                for(int idx = 0; idx < n; ++idx){	
                    //just check if inside hotspot location at all - will return 1 if button, 2 if crank or slider (denoting bar or thumb)
                    // KinUIComponent* cmp = Kin2dUICmp[idx].get();
                    KinUIComponent* cmp = NULL;
                    if (idx < m) {
                        cmp = Kin2dUICmp[idx].get();
                    } else {
                        cmp = manualUICmp[idx - m];
                    }

                    // If not slider and state is not changed
                    if (dynamic_cast<KinUISlider*>(cmp) == NULL &&
                        oldHandState == evnt) {
                        continue;
                    }
                    int clkHSVal = (cmp->isInsideHotSpot(x,y));
                    // int clkHSVal = (cmp->isInside(x,y));

                    if((0 != clkHSVal) || (cmp->checkDragByThisCntrl(hand))){                                     //verify that this hand is the hand generating the event
                    //if(0 != clkHSVal){                                                                                          //verify that this hand is the hand generating the event
                        // clog<<"hand : "<< (KC_LEFT == hand ? " Left" : " Right")<< (hndDragEvent ? " dragged " : " clicked ")<<"in object IDX: "<<idx<<" event : "<<evnt<<" clickEvent : "<<hndClickEvent<<" hotspot val : "<<clkHSVal<<" x,y : ("<<x<<", "<<y<<")"<<std::endl;
                        if(hndClickEvent){		
                            cmp->setClicked(x,y,clkHSVal, hand);
                            if(0 != clkHSVal){
                                this->KinHands[hand].setFlags(_KHnd_HAND_DRAG_OBJ,true);
                                this->KinHands[hand].setFlags(_KHnd_HAND_CLCKD,true);
                            }    //if being dragged by this input, set to true
                            if(hndDragEvent){
                                int resD = cmp->setDragged(hndDragEvent,x,y, hand);       
                                if (resD != -1){
                                    //clog<<(KC_LEFT == hand ? "Left" : "Right")<<" hand dragging event : "<<hndDragEvent<<std::endl;
                                    int tmpX = x, tmpY = y;
                                    cmp->stickHandToClick(tmpX,tmpY);                       //tmpX,tmpY get set with position where hand should be moved to while dragging
                                    this->KinHands[hand].setNewHandLoc2d(tmpX, tmpY);                   //set new location for this hand
                                }                                                                       //force hand to stay inside object while click/grab engaged                               
                            }
                        } else 	{	cmp->clearClicked(x,y, hand);}
                        objFound = true;	
                    } else {     cmp->clearClicked(x,y, hand); }												
                }//for each object
                this->KinHands[hand].setFlags(_KHnd_HAND_IN_OBJ, objFound);
                if((0==evnt) || (!(objFound))){       
                    this->KinHands[hand].setFlags(_KHnd_HAND_DRAG_OBJ,false);
                }    //clear drag flag corresponding to this hand if we're not in an object    
            }//if event from mouse else from kin hand
            return objFound;
        }//checkUIElements

        ////////
        // updates for each stream and IK - locally referenced
        ////////

        void KinectController::runIKSolver(){
            int iters = 0;
            double fq = kinIKSolver->calcFq();
            while((fq  > KinIK_eps * mNumMarkers) && (iters < KinIK_numIters)){  	
                kinIKSolver->Solve(fq);
                fq = kinIKSolver->calcFq();
                ++iters;
            }//while
        }//runAgentSolver

        //receive kinect skel joint locs from kinect and send them to IK solver engine
        void KinectController::updateMarkersAndIK(){
            //need to copy markers to local container so that they don't change as they are being used
            kinSkelMarkerPos = kinHNDLR->getLastKnownSkelJointLocs();
            kinSkelNoisyMarkers = kinHNDLR->getKinSkelNoisyMarkers();
            if(this->KC_flags[_KC_PROC_IK]){
                //set values in IK Solver - needs to be vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>
                kinIKSolver->setMarkerData(kinSkelMarkerPos);
                //perform IK
                if(!(kinIKSolver->runSolver)){	StartKinIKSolver();		}	//TODO					//if not supposed to be running, check if ready to run, and if so then start running		
                if(kinIKSolver->runSolver){		runIKSolver();			}							//if ready to run, and supposed to, then run
            }
            if(kinHNDLR->kinSkelHandlerAvail()){	loadKinSkelMarkerInfo();	}					//send most recent IK joint pos to kin code, if valid handler available		
        }//updateMarkersAndIK

        //process speech if it has occurred
        void KinectController::audioUpdate(){							
            kinHNDLR->processSpeechEvents();
            kinSpeechRecog = kinHNDLR->getSpchRecogResult();
        }//audioUpdate

        //clear speech recognition text once acted upon
        void KinectController::clearSpeechRecogVals(){
            kinSpeechRecog = "";														            //to prevent multiple calls of handler
            kinHNDLR->clearSpeechVals();
        }//clearSpeechRecogVals

        //send most recent ik'ed avatar marker locations back to kinect skeleton handler
        void KinectController::loadKinSkelMarkerInfo(){
            //vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> tmpCoachIKHandles(20);
            vector<string> tmpCoachIKHndlNames(20);
            //get global position of skeleton upon loadin to set up IK framework in kinect skel code
            for(int idx = 0; idx < avatarIKHandles.size(); ++idx){	
                avatarIKHandles[idx] = kinSkel->getMarker(idx)->getWorldCoords();//mHandleList[idx]->mGlobalPos;
                stringstream ss;
                ss<<kinSkel->getMarker(idx)->getName();
                tmpCoachIKHndlNames[idx] = ss.str();
            }
            kinHNDLR->buildSkelFrameFromSkelAvatar(avatarIKHandles,tmpCoachIKHndlNames);
        }//loadKinSkelMarkerInfo

            //train net via nested loops of back propagation
        //resWghts : 10 3d arrays, each a net with per layer, per percep, list of weights
        void KinectController::buildNNet(){
            vector<vector<int>> inVec;                  //populate with training and test data
            vector<vector<int>> outVec;
            //for (int i = 0; i < numNets; ++i){
            //    double totErr = 99999;
            //    double netAvgWghtMod = 0;
            //    kNet.randomizeWeights();
            //    while(netAvgWghtMod > minWghtMod){  kNet.backPropLearning(inVec, outVec, totErr, netAvgWghtMod);  }                
            //}//train numNets nets
        }//buildNNet


        /////////////////
        /// getters/setters
        /////////////////

        //setters from XML code - replace hardcoded values
        bool KinectController::setKinectHandlerFlag(int idx, bool val){          kinHNDLR->setFlag(idx,val);      return true; }
        bool KinectController::setKinAudioHandlerFlag(int idx, bool val){        bool resCode = kinHNDLR->setKinAudioHandlerFlag(idx,val);    return resCode; }
        bool KinectController::setKinDepthHandlerFlag(int idx, bool val){        bool resCode = kinHNDLR->setKinDepthHandlerFlag(idx,val);    return resCode; }
        bool KinectController::setKinImageHandlerFlag(int idx, bool val){        bool resCode = kinHNDLR->setKinImageHandlerFlag(idx,val);    return resCode; }
        bool KinectController::setKinInteractHandlerFlag(int idx, bool val){     bool resCode = kinHNDLR->setKinInteractHandlerFlag(idx,val); return resCode; }
        bool KinectController::setKinSkelHandlerFlag(int idx, bool val){         bool resCode = kinHNDLR->setKinSkelHandlerFlag(idx,val);     return resCode; }

        //default state vars, overridden by individual objects - TODO
        bool KinectController::setKinectUIComponentFlag(int idx, bool val){         bool resCode = true;  return resCode;      }
                                                                         

        //end setters from XML code - replace hardcoded values

        vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>* KinectController::GetMarkers(){return &kinSkelMarkerPos;}
        Eigen::Vector3d KinectController::GetMarkerPos(int markerIndex){
            if(!(kinHNDLR->kinValidSkelAvail()) || (markerIndex >= mNumMarkers) || (markerIndex < 0)){	return Eigen::Vector3d(0,0,0);}	//if no valid kinect/skel data available or marker index is out of range
            else {																						return kinSkelMarkerPos[markerIndex];	}//if idx legal or not
        }//getMarkerPos

        int KinectController::GetMarkerCount(){   return mNumMarkers;}
        string KinectController::GetKinSpeechRecog(){return kinSpeechRecog;}
        vector<string> KinectController::GetSkelJointVals() {return skelJointStrs;}

        //start kinect IK Solver
        void KinectController::StartKinIKSolver(){	
            if(kinIKSolver->isReadyToSolve()){	kinIKSolver->runSolver = true;}
            clog<<"Kinect Controller : Kin IK solver ready : "<<kinIKSolver->isReadyToSolve()<<std::endl;
        }//SetKinIKSolver

        //should only be called by instancing class, if there is one
        bool KinectController::parseSpeechCommand(){
            if (kinSpeechRecog == "IK MODE"){
                clog<<"KinectController : Change to IK mode"<<endl;
                setFlag(_KC_PROC_IK, true);
                clearSpeechRecogVals();
                return true;
            } else if (kinSpeechRecog == "UI MODE"){
                clog<<"KinectController : Change to UI mode"<<endl;
                setFlag(_KC_PROC_HANDS_2D, true);
                clearSpeechRecogVals();
                return true;
            } else if (kinSpeechRecog == "3D MODE"){
                clog<<"KinectController : Change to 3D mode"<<endl;
                setFlag(_KC_PROC_HANDS_3D, true);
                clearSpeechRecogVals();
                return true;
            } else if (kinSpeechRecog == "GRAB MODE ON"){
                clog<<"KinectController : Enable Depth-based hand grab for dragging"<<endl;
                if(KC_flags[_KC_PROC_HANDS_2D]){
                    setFlag(_KC_PROC_2D_DGRAB, true);
                }           //only turn on if in 2d ui mode
                clearSpeechRecogVals();
                return true;

            } else if (kinSpeechRecog == "GRAB MODE OFF"){
                clog<<"KinectController : Disable Depth-based hand grab for dragging"<<endl;
                setFlag(_KC_PROC_2D_DGRAB, false);
                clearSpeechRecogVals();
                return true;

            } else {
                return false;
            }
        }//parseSpeechCommand


        void KinectController::StopKinIKSolver(){
            clog<<"Kinect Controller : Kin IK Solver Calculations Terminated" << endl;
            kinIKSolver->runSolver = false;	
        }
        //put inline if possible
        inline void KinectController::setFlag(int idx, bool val){
            KC_flags[idx] = val;
            switch(idx){
            case _KC_PROC_HANDS_2D	: {
                if(val){
                    clog<<"set 2d hands mode"<<std::endl;
                    setDpthHandState(true,true);
                    KC_flags[_KC_PROC_HANDS_3D] = false;
                    KC_flags[_KC_DISP_UI_2D] = true;
                    KC_flags[_KH_DPTH_DISP_HND] = true;
                    KC_flags[_KH_SKEL_CALC_HND] = true;
                    KC_flags[_KC_DISP_UI_IK] = false;
                    KC_flags[_KC_PROC_IK] = false;}					
                break;}									//show 2d hands to interact with gui layer
            case _KC_PROC_HANDS_3D	: {
                if(val){
                    clog<<"set 3d hands mode"<<std::endl;
                    setDpthHandState(true,true);
                    KC_flags[_KH_DPTH_DISP_HND] = true;
                    KC_flags[_KH_SKEL_CALC_HND] = true;
                    KC_flags[_KC_PROC_HANDS_2D] = false;
                    KC_flags[_KC_DISP_UI_2D] = false;
                    KC_flags[_KC_DISP_UI_IK] = false;
                    KC_flags[_KC_PROC_IK] = false;}					
                break;}									//show 3d hands to interact skel/scene objs
            case _KC_PROC_IK		: {
                if(val){	
                    clog<<"set IK  mode"<<std::endl;
                    setDpthHandState(false,false);
                    KC_flags[_KC_PROC_HANDS_2D] = false;
                    KC_flags[_KC_DISP_UI_IK] = true;
                    KC_flags[_KC_DISP_UI_2D] = true;
                    KC_flags[_KH_DPTH_DISP_HND] = false;
                    KC_flags[_KH_SKEL_CALC_HND] = false;
                    KC_flags[_KC_PROC_HANDS_3D] = false;

                    KC_flags[_KC_DISP_UI_2D] = true;
                    cout << "set draw2d: " << KC_flags[_KC_DISP_UI_2D] << endl;
                }
                break;}									//whether or not we should process IK - want to turn off and freeze result when processing Kinect UI commands
            //case _KC_MOUSE_USE : {                    //move to hand class TODO
            //    KC_flags[_KC_CHNG_HANDS_2D] = !val;                
            //    break;}
            //case _KC_CHNG_HANDS_2D : {					//hands have changed from most recent previous values
            //    KC_flags[_KC_MOUSE_USE] = !val;
            //    break;}
            default : {break;}
            }//switch - set aux values based on flag settings
        }
        void KinectController::SetMarkerPos(int markerIndex, Eigen::Vector3d& pos){    kinSkelMarkerPos[markerIndex] = Eigen::Vector3d(pos);  }
        void KinectController::SetShowModel(bool val){setFlag(_KC_DISP_AVATAR, val);}
        void KinectController::SetShowMarkers(bool val){setFlag(_KC_DISP_MRKRS, val);}

            //kinHandler xml var setters
        bool  KinectController::setKinLibGlobalConstants(
            int tmp_RCD,                                              //_RCD              type:int        cmt:reconnect retry countdown timer
            long tmpkinMaxTilt,                                       //kinMaxTilt        type:long       cmt: max tilt angle
            long tmpkinMinTilt,                                       //kinMinTilt        type:long       cmt: min tilt angle
            wstring& tmpGrammarFileNameStr,                            //GramarFileNameStr type:string     cmt:file name for speech recognition grammar
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
            float tmpUIobj_CrankSens,                                 //UIobj_CrankSens    type:float     cmt:sensitivity of crank bar drag - 0.0-1.0
            float tmpLHandDragSens,                                   //LHandDragSens       type:float    cmt:left hand sensitivity setting for when dragging
            float tmpRHandDragSens,                                    //RHandDragSens       type:float    cmt:right hand sensitivity setting for when dragging
            float tmpLHandPushSensIn,                                 //type:float      cmt:left hand push in sensitivity to detect click
            float tmpRHandPushSensIn,                                 //type:float      cmt:left hand push in sensitivity to detect click
            float tmpLHandPushSensOut,                                //type:float      cmt:right hand pull back sensitivity to disengage click
            float tmpRHandPushSensOut,                                 //type:float      cmt:right hand pull back sensitivity to disengage click
            float tmpUIobj_MinFontScale,                              //type:float      cmt:min value for font scaling for ui object labels
            float tmpUIobj_MaxFontScale                               //type:float      cmt:max value for font scaling for ui object labels
           )
        {
            _RCD             = tmp_RCD;                                      
            kinMaxTilt       = tmpkinMaxTilt;                               
            kinMinTilt       = tmpkinMinTilt;                               
            GrammarFileName  = wstring(tmpGrammarFileNameStr); 

            DP_GRIPH_THRESH  = tmpgripH;
            DP_GRIPW_THRESH  = tmpgripW;
            DP_GRIPD_THRESH  = tmpgripD;
            for(int i = 0; i < 4; ++i){
                LHandBx[i]   = tmpLHandBx[i];                                
                RHandBx[i]   = tmpRHandBx[i];  
            }
            skelSmoothing    = tmpskelSmoothing; 
            MX_CNT_FLR_Y     = tmpMX_CNT_FLR_Y;  
            for(int i = 0; i<3; ++i){
                SK_JNT_MULT[i]    = tmpSK_JNT_MULT[i];                          
                SK_JNT_OFF[i]     = tmpSK_JNT_OFF[i];
                jntSz[i]          = tmpjntSz[i];  
                for(int j=0; j<3; ++j){         kinSkelDisp[i][j]      = tmpkinSkelDisp[i][j];         }
            }
            jntPosAraDpth       = tmpjntPosAraDpth;                             
            jntNumMaxInterp     = tmpjntNumMaxInterp;                           
            TRY_KIN_RCN         = tmpTRY_KIN_RCN;                               
            KC_hndEps_2D        = tmpKC_hndEps_2D;                              
            KinPshDist          = tmpKinPshDist;                              
            KinIK_eps           = tmpKinIK_eps;                               
            KinIK_minPrtrb      = tmpKinIK_minPrtrb;                          
            KinIK_numIters      = tmpKinIK_numIters;                            
            KinIK_weights       = Eigen::VectorXd(tmpKinIK_weights);
            KinSldrBrdr         = tmpKinSldrBrdr;                             
            UIobj_DragSens      = tmpUIobj_DragSens;  
            UIobj_CrankSens     = tmpUIobj_CrankSens; 
            LHandDragSens       = tmpLHandDragSens;
            RHandDragSens       = tmpRHandDragSens;
            LHandPushSensIn     = tmpLHandPushSensIn;             
            RHandPushSensIn     = tmpRHandPushSensIn;             
            LHandPushSensOut    = tmpLHandPushSensOut;            
            RHandPushSensOut    = tmpRHandPushSensOut;     
            UIobj_MinFontScale  = tmpUIobj_MinFontScale; 
            UIobj_MaxFontScale  = tmpUIobj_MaxFontScale;
            return true;
        }//setKinLibGlobalConstants

        //kinect namespace-scope variables populated by xml, set with default values below
        int _RCD = 600;                                                                         //type:int        cmt:reconnect retry countdown timer
        long kinMaxTilt = 27;                                                                   //type:long       cmt: max tilt angle
        long kinMinTilt = -27;                                                                  //type:long       cmt: min tilt angle
        float DP_GRIPH_THRESH = .75f;                                                           //type:float      cmt: threshold plane for grip detection - height
        float DP_GRIPW_THRESH = .95f;                                                           //type:float      cmt: threshold plane for grip detection - width
        float DP_GRIPD_THRESH = 1.25f;                                                          //type:float      cmt: threshold plane for grip detection - depth
        int LHandBx[4] = {70, -65, 100, -95};                                                   //type:int[4]     cmt:ara of vals for left hand detect box : {max x, min x-max x, max y, min y-max y}
        int RHandBx[4] = {70, -65, 100, -95};                                                   //type:int[4]     cmt:ara of vals for right hand detect box : {max x, min x-max x, max y, min y-max y}
        float HndTrkBxMult[6] = {0,3,.5,1.5,0,3};                                               //type:float[6]   cmt:ara of multipliers for relative hand to root tracking box
        int MX_CNT_FLR_Y = 30;                                                                  //type:int        cmt:max number of consecutive frames of same floor depth counted (if kinects interpolated floor changes, decrement until 0 then change, to act as smoothing)
        float SK_JNT_MULT[3]= {1.0f, 1.0f, -1.0f};                                              //type:float[3]   cmt:multiplier used when converting joint coords from skel space to display space
        float SK_JNT_OFF[3]= {0.0f, 0.0f, 1.0f};                                                //type:float[3]   cmt:offset used when converting joint coords from skel space to display space
        double jntSz[3]= {.02, .02, .02};                                                       //type:double[3]  cmt:size of skel joint ara for display
        int jntPosAraDpth = 4;                                                                  //type:int        cmt:how many previous values for jnt pos,vel,accel we will keep around
        int jntNumMaxInterp = 10;                                                               //type:int        cmt:maximum interpolations before we just use most recent avatar value for joint ara location
        int TRY_KIN_RCN = 100000;                                                               //type:int        cmt:kinect controller reconnect retry countdown
        int KC_hndEps_2D = 1;                                                                   //type:int        cmt:threshold amount hands need to move in xy to be detected as moving in screen space
        float KinPshDist = 100;                                                                 //type:float      cmt: arbitrary; defines delta z to be considered push
        float KinIK_eps = .001f;	                                                            //type:float      cmt:IK eps amt
        float KinIK_minPrtrb = .01f;		                                                    //type:float      cmt:minimum timestep amount we will allow for any single iteration
        int KinIK_numIters = 5;				                                                    //type:int        cmt:number of IK iterations to run per timer cycle
        Eigen::VectorXd KinIK_weights;                                                          //type:Eigen::VectorXd    cmt:weights for joints in IK skel, idx'ed by kin joint ara idx - currently 20 joints in kin skel
        float KinSldrBrdr = 5;                                                                  //type:float      cmt:graphical border around slider object
        float UIobj_DragSens = .5f;                                                             //type:float      cmt:sensitivity of slider bar drag - 0.0-1.0
        float UIobj_CrankSens = .02f;                                                           //type:float      cmt:sensitivity of crank bar drag - 0.0-1.0
        float LHandDragSens = 10;                                                               //type:float      cmt:left hand sensitivty setting for when dragging
        float RHandDragSens = 10;                                                               //type:float      cmt:right hand sensitivty setting for when dragging
        float LHandPushSensIn = .5f;                                                            //type:float      cmt:left hand push in sensitivity to detect click
        float RHandPushSensIn = .5f;                                                            //type:float      cmt:left hand push in sensitivity to detect click
        float LHandPushSensOut = .4f;                                                           //type:float      cmt:right hand pull back sensitivity to disengage click
        float RHandPushSensOut = .4f;                                                          //type:float      cmt:right hand pull back sensitivity to disengage click
		wstring GrammarFileName = WRTQL8_DATA_PATH L"grammar/SpeechGrammar.grxml";		        //use this file to hold primary grammar
        NUI_TRANSFORM_SMOOTH_PARAMETERS skelSmoothing = {0.5f, 0.1f, .5f, 0.1f, 0.1f};          //type:struct NUI_TRANSFORM_SMOOTH_PARAMETERS     cmt:used for kinect sdks internal calc of skeleton motion smoothing - see sdk for params
        float kinSkelDisp[3][3] = {{1.0f,0.0f,0.0f}, {1.0f,2.0f,0.0f}, {-1.0f,0.0f,0.0f}};      //type:float[3][3]        cmt:translation on screen for drawing raw kinect joint ara data noisy kin, filtered kin, avatar(postIK)

        double NNalpha = .1;                                                                    //timestep for NN
        int randSfx;
        //font display values - min and max scaling value
        float UIobj_MinFontScale = .01f;                                                        //type:float      cmt:min value for font scaling for ui object labels
        float UIobj_MaxFontScale = .3f;                                                         //type:float      cmt:max value for font scaling for ui object labels
    }//namespace kinect

}//namespace rtql8
