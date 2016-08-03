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
#include "KinCntrlHand.h"


namespace rtql8{
    namespace kinect{

        KinCntrlHand::KinCntrlHand(KinectController* _kc, int _hnd):
            hand(_hnd), kc(_kc), handState(0), oldHandState(0), imgState(0), pushProgress(0),lastObjID(-1), KHand_flags(KHnd_numFlags, false),initLoc(2,0),
            hBxLowMlt(3,0), hBxHiMlt(3,0), clr2d(4,1.0), dim2d(2,1.0)
        {
            //set initial positions and color : left hand - blue , right hand red
            if(KC_LEFT == hand) {   
                initLoc[0] = 500;//kc->winX * 2/5;
                clr2d[0] = 0; 
                dim2d[0] = -1 * dims[1][1];
                zSensMultIn = LHandPushSensIn;
                zSensMultOut = LHandPushSensOut;
                dragSens = LHandDragSens;
                    //low vals for tracking zone box - x
                hBxLowMlt[0] = -1 * HndTrkBxMult[1];
                    //high vals for tracking zone box
                hBxHiMlt[0] = -1 * HndTrkBxMult[0];
            } 
            else {                           
                initLoc[0] = 600; //kc->winX * 3/5;
                clr2d[2] = 0;  
                dim2d[0] = dims[1][1];
                zSensMultIn = RHandPushSensIn;
                zSensMultOut = RHandPushSensOut;
                dragSens = RHandDragSens;
                    //low vals for tracking zone box
                hBxLowMlt[0] = HndTrkBxMult[0];
                    //high vals for tracking zone box
                hBxHiMlt[0] = HndTrkBxMult[1];
            }
            initLoc[1] = 400;//(kc->winY / 2);
            //low and high vals for y and z same for both hands
            hBxLowMlt[1] = HndTrkBxMult[2];
            hBxHiMlt[1] = HndTrkBxMult[3];
            hBxLowMlt[2] = HndTrkBxMult[5];
            hBxHiMlt[2] = HndTrkBxMult[4];
            clr2d[1] = 0;   
            dim2d[1] = -dims[1][1];
            initHand();                         //initializes arrays but needs to be called again once window is instantiated to set appropriate location for hands to start
        }//cnstrctr

        ///////////////
        //draw routines
        ///////////////
        //draw hand for 2d UI
        void KinCntrlHand::draw2d(){
            glPushMatrix();
                glTranslatef(gHandLoc(0),gHandLoc(1),0);
                drawHand2d();
            glPopMatrix();
        }//draw

        //draw hand in 3d space for 3d ui
        void KinCntrlHand::draw3d(){
            glPushMatrix();
                glTranslatef(spcHandLoc(0),spcHandLoc(1), spcHandLoc(2));
                drawHand3d();
            glPopMatrix();
        }//draw

        ////draws hand flat on the screen, either open, push or grip
        void KinCntrlHand::drawHand2d(){
            bool left = (KC_LEFT==hand? true : false);
            dim2d[1] = (1==handState ? .5f : 1) * -dims[1][1];                         //set dim[1] == y dimension multiplier to .5 if press or 1 if not press
            glPushMatrix();
            glPolygonMode(GL_FRONT_AND_BACK,  GL_FILL);			
            glPushMatrix();
                glTranslatef(dim2d[0]*3, 0,0);
                drawPushProgress();
            glPopMatrix();

            glColor4f(clr2d[0],clr2d[1],clr2d[2],(KHand_flags[_KHnd_HAND_TRACKED] ? 1 : .1 ) * clr2d[3]);
            //draw palm
            glBegin(GL_TRIANGLE_FAN);
                glVertex2f(0,0);
                for(int idx = 0; idx < KinNmPlmPts; ++idx){	glVertex2f(pMult[idx][0] * dim2d[0], pMult[idx][1] * dim2d[1]);	}
                glVertex2f(pMult[0][0] * dim2d[0], pMult[0][1] * dim2d[1]);
            glEnd();
            //draw fingers
            if((0 == handState) || (1 == handState)){	//draw open hand or pressing hand
                for(int fng = 0; fng < 5; ++fng){
                    glBegin(GL_TRIANGLE_STRIP);
                        for(int idx = 0; idx < KinNmOFPts; ++idx){	glVertex2f(ofMult[fng][idx][0] * dim2d[0], ofMult[fng][idx][1] * dim2d[1]);	}//for each point
                    glEnd();
                }//for each finger				
            } else if(2 == handState){	//draw closed hand
                for(int fng = 0; fng < 5; ++fng){
                    glBegin(GL_TRIANGLE_STRIP);
                        for(int idx = 0; idx < KinNmCFPts; ++idx){	glVertex2f(cfMult[fng][idx][0] * dim2d[0], cfMult[fng][idx][1] * dim2d[1]);	}//for each point
                    glEnd();
                }//for each finger
            }//draw fingers

            ////draw center
            //glColor4f(0,0,0,1);
            //glBegin(GL_TRIANGLE_FAN);
            //	glVertex2f(0.1f, 0.1f);
            //	glVertex2f(0.1f, -0.1f);
            //	glVertex2f(-0.1f, -0.1f);
            //	glVertex2f(-0.1f, 0.1f);					
            //glEnd();

            //drawKinDpthHandImage(hand,0,0,0,1,1,1);               //draw depth hand superimposed over hand image
            glPopMatrix();
        }//drawHand2d
        
        //draw bar with percent from full push - called from drawHand2d - dims holds where bar should go and how big it should be - idx : 0->xloc 1->yloc of upper corner, 2->thick, 3->length
        void KinCntrlHand::drawPushProgress(){
            float prg = pshBarlen * pushProgress/100.0f;		//pushProgress is value from 0-100
            glPushMatrix();
            glColor4f(.5f,.5f,.5f,(KHand_flags[_KHnd_HAND_TRACKED] ? 1 : .1 ) *1.0f);							
            glBegin(GL_TRIANGLE_STRIP);
                glVertex2f(dims[0][0],dims[0][1]);
                glVertex2f(dims[1][0],dims[1][1]);
                glVertex2f(dims[0][0],dims[0][1] - pshBarlen);
                glVertex2f(dims[1][0],dims[1][1] - pshBarlen);
            glEnd();

            glColor4f(0,1.0f,.8f,(KHand_flags[_KHnd_HAND_TRACKED] ? 1 : .1 ) *1);							
            glBegin(GL_TRIANGLE_STRIP);
                glVertex2f(dims[0][0],dims[0][1]);
                glVertex2f(dims[1][0],dims[1][1]);
                glVertex2f(dims[0][0],dims[0][1] - prg);
                glVertex2f(dims[1][0],dims[1][1] - prg);
            glEnd();
            glPopMatrix();
		
        }//drawPushProgress

        //location will be scaled location from depth space, with minimum depth being near root location and maximum depth being ~.75 m away, and x-y location 
        //being scaled from x-y location in depth space to appropriate location in space
        void KinCntrlHand::drawHand3d(){
            //bool left = (KC_LEFT==hand? true : false);
            //float dim[2] = {(left ? -1 : 1) * dims[1][1], -dims[1][1]};
            //ShapeEllipsoid temp(Eigen::Vector3d(jntSz[0], jntSz[1], jntSz[2]), 0.0);
            //ball at center of hand
            glPushMatrix();
                //temp.draw(mRI);																//if render interface not defined/null, will not draw sphere
                drawHand2d();
            glPopMatrix();
        }//drawHand3d

        //////////
        //// UI building and event handling
        //////////
        //called from kinect controller
        void KinCntrlHand::handleHand(Eigen::VectorXd& handValLocAra, int gripState){			            //currently 2d only
            if(handValLocAra[2] > -100){														            //z == -100 used as sentinel value if skel handler is not instantiated or if no valid data sent - hands should never be in -100 z in skel space
                KHand_flags[_KHnd_HAND_TRACKED] = true;
                setNewHandVals2d(handValLocAra, gripState);}
            else {		                                                                                   
                // cout<<"hand not tracked"<<endl;
                KHand_flags[_KHnd_HAND_TRACKED] = false;
                KHand_flags[_KHnd_HAND_CHNG_2D] = false;	
            }						    

            if(KHand_flags[_KHnd_HAND_TRACKED]){
                // if((0==gHandLoc(0)) && (0== gHandLoc(1)) ){  cout<<(*this)<<"-------NOT CHANGING---------"<<endl;}
                //NOTE :  Z here if state has changed for hands, or hands are gripping, then process events
                if((2 == handState) || (1 == handState) || (chkStateChange())) { kc->checkUIElements(gHandLoc(0), gHandLoc(1), gHandLoc(2), false, hand,  handState);  }
                setOldHandVals();
            }
        }//handleHands

            //calculate whether pushing and the progress of push/release
        void KinCntrlHand::calc2dPush(){
            if(0 == handState){		                    pushProgress += (zSensMultIn * - (gHandLoc(2) - oldScnHandLoc(2)));}// - (abs(gHandLoc(0) - oldScnHandLoc(0)) + abs(gHandLoc(1) - oldScnHandLoc(1)));	}    //open hand, check if pushing
            else if (1 == handState) {	                pushProgress += (zSensMultOut * -(gHandLoc(2) - oldScnHandLoc(2)));}	            //click hand, check if withdrawing only, to enable drag function
            else {						                pushProgress = 0;}                                                                  //don't use push with grip
            if(pushProgress < 0 ){                      pushProgress = 0; handState = 0;}																											
            else if (pushProgress >= 100){              pushProgress = 100; handState = 1;}                                                 //bound click progress/reset click 
        }//calc2dPush

        ////determine new hand locations and events for 2d ui
        void KinCntrlHand::setNewHandVals2d(Eigen::VectorXd& handValLocAra, int gripState){
            //handle interaction with UI
            //location on screen plane for both hands
            handLocPos<<handValLocAra(0),handValLocAra(1),handValLocAra(2);					            //x,y,depth/z in depth ara space - used for comparison
            rootLocPos<<handValLocAra(3),handValLocAra(4),handValLocAra(5);
            rShldrLocPos<<handValLocAra(6),handValLocAra(7),handValLocAra(8);

            //build box for hands based on relative hand location to root - 
            //from half the distance between root and shoulder above the root to 1.5 times this distance above the root, in y
            //  and from the root to 2x the distance from the root to the shoulder in x.
            //  we are assuming the user is standing with kinect view perpendicular to coronal plane (through shoulders and root of body)
            double   rtShldrDelX = abs(rootLocPos(0) - rShldrLocPos(0)),  
                    rtShldrDelY = abs(rootLocPos(1) - rShldrLocPos(1)),  
                    lowHandPosY = rootLocPos(1) + hBxLowMlt[1] * rtShldrDelY,        //for using skeleton, where y increases upwards
                    highHandPosY = lowHandPosY + hBxHiMlt[1] * rtShldrDelY;

            //map hands from this physical zone to screen dimensions
            // cout << handLocPos(0) << " " << handLocPos(1) << " " << handLocPos(2) << endl;
             newScnHandLoc(0) = mapSkelSpcToScrn(rootLocPos(0) + (hBxLowMlt[0] * rtShldrDelX), rootLocPos(0) + (hBxHiMlt[0] * rtShldrDelX), 0, kc->winX, handLocPos(0));		
             newScnHandLoc(1) = mapSkelSpcToScrn(lowHandPosY, highHandPosY, kc->winY, 0, handLocPos(1));
            // //map skel depth to screen-scale coords - want similar scale vals as x and y so that we can more accurate determine push
            newScnHandLoc(2) = mapSkelSpcToScrn(rootLocPos(2)-(hBxLowMlt[2]*rtShldrDelX), rootLocPos(2)-(hBxHiMlt[2]*rtShldrDelX), 0, kc->winX, handLocPos(2));

            //map hands from this physical zone to screen dimensions
            //newScnHandLoc(0) = mapSkelSpcToScrn(-0.64, 0.64, 0, kc->winX, handLocPos(0));		
            //newScnHandLoc(1) = mapSkelSpcToScrn(-0.32, 0.32, kc->winY, 0, handLocPos(1));
            //map skel depth to screen-scale coords - want similar scale vals as x and y so that we can more accurate determine push
            //newScnHandLoc(2) = mapSkelSpcToScrn(rootLocPos(2)-(hBxLowMlt[2]*rtShldrDelX), rootLocPos(2)-(hBxHiMlt[2]*rtShldrDelX), 0, kc->winX, handLocPos(2));
            
            //if a hand is dragging an object currently, use displacement method to determine where hand moves, else use mapping
            if(KHand_flags[_KHnd_HAND_DRAG_OBJ]){    
                //clog<<"dragging "<<(KC_LEFT == hand ? "Left" : "Right" )<<" hand"<<std::endl;
                Eigen::Vector3d tmp(handLocPos - oldHandPos);
                //Eigen::Vector3d tmp(newScnHandLoc(0) - oldScnHandLoc(0),newScnHandLoc(1) - oldScnHandLoc(1), newScnHandLoc(2) - oldScnHandLoc(2));
                tmp.normalize();
                tmp = tmp * dragSens;      
                gHandLoc = gHandLoc + Eigen::Vector3i(int(tmp(0)),int(tmp(1)),newScnHandLoc(2));
                gHandLoc(2) = newScnHandLoc(2);                                                         //keep depth calc, irrelevant for dragging
            }
            else {                   gHandLoc = Eigen::Vector3i(newScnHandLoc);          }              //standard processing
                
            boundHandLoc();
                  
            if ((0 == handState) && 
                (abs(oldScnHandLoc(0) - newScnHandLoc(0)) < KC_hndEps_2D) && 
                (abs(oldScnHandLoc(1) - newScnHandLoc(1)) < KC_hndEps_2D) && 
                (abs(oldScnHandLoc(2) - newScnHandLoc(2)) < KC_hndEps_2D)) {	
                KHand_flags[_KHnd_HAND_CHNG_2D] = false;								                //hands match most recent values and not exerting an event
            } else {
                KHand_flags[_KHnd_HAND_CHNG_2D] = true;								                
                //determine hand push
                calc2dPush();
				//check if depth-driven grip happening - only if flag set and if not already determined to be clicking
                if ((-1 != gripState) && (1 != handState)){
                    handState = gripState;                                                             //if not pushing already, use depth-determined grip state if present
                }
            }//if changed position       				
        }//setNewHandVals

        //bound hand location to be within screen boundaries
        void KinCntrlHand::boundHandLoc(){
            if(gHandLoc(0) < 0){gHandLoc(0) = 0;}
            if(gHandLoc(1) < 0){gHandLoc(1) = 0;}
            if(gHandLoc(0) > kc->winX){gHandLoc(0) = kc->winX;}
            if(gHandLoc(1) > kc->winY){gHandLoc(1) = kc->winY;}
        }

        //set this round's location values as "old" values, to be used for comparisons next round - this is after processing hand motion for a single frame
        void KinCntrlHand::setOldHandVals(){
            if(KHand_flags[_KHnd_HAND_CHNG_2D]){
                oldHandState = handState;
                oldgHandLoc = Eigen::Vector3i(gHandLoc);
                oldScnHandLoc = Eigen::Vector3i(newScnHandLoc);
                oldHandPos = Eigen::Vector3d(handLocPos);
                oldRootPos = Eigen::Vector3d(rootLocPos);
                oldrShldrPos = Eigen::Vector3d(rShldrLocPos);
                KHand_flags[_KHnd_HAND_CHNG_2D] = false;												//hands now match most recent values
            }//no neeed to save if hands haven't moved
        }//setOldHandVals

        void KinCntrlHand::setFlags(int idx, bool val){
            this->KHand_flags[idx] = val;
            switch (idx){
            case _KHnd_HAND_CHNG_2D :{break;} 
            case _KHnd_HAND_DRAG_OBJ :{break;}	
            case _KHnd_HAND_IN_OBJ :{break;}
            case _KHnd_HAND_TRACKED : {break;}
            default : {break;}
            }//switch
        }//setFlags

        std::ostream& operator<<(std::ostream& out, const KinCntrlHand& hnd){
            // out<<(KC_LEFT == hnd.hand ? "Left Hand Location : (" : "Right Hand Location : (")<<hnd.gHandLoc(0)<<", "<<hnd.gHandLoc(1)<<", "<<hnd.gHandLoc(2)<<") State : "<<hnd.handState<<"Previous location : ("<<hnd.oldgHandLoc(0)<<", "<<hnd.oldgHandLoc(1)<<", "<<hnd.oldgHandLoc(2)<<") Previous State : "<<hnd.oldHandState<<" Most Recent Object ID : "<<hnd.lastObjID<<endl;
            // out<<"Curr Mapped skel->scn Hand Location : ("<<hnd.newScnHandLoc(0)<<", "<<hnd.newScnHandLoc(1)<<", "<<hnd.newScnHandLoc(2)<<")  Previous Mapped skel->scn location : ("<<hnd.oldScnHandLoc(0)<<", "<<hnd.oldScnHandLoc(1)<<", "<<hnd.oldScnHandLoc(2)<<")"<<endl;
            // out<<"Curr skel location : Hand : ("<<hnd.handLocPos(0)<<", "<<hnd.handLocPos(1)<<", "<<hnd.handLocPos(2)<<") Root : ("<<hnd.rootLocPos(0)<<", "<<hnd.rootLocPos(1)<<", "<<hnd.rootLocPos(2)<<") Shldr : ("<<hnd.rShldrLocPos(0)<<", "<<hnd.rShldrLocPos(1)<<", "<<hnd.rShldrLocPos(2)<<")"<<endl;
            // out<<"Old skel location : Hand : ("<<hnd.oldHandPos(0)<<", "<<hnd.oldHandPos(1)<<", "<<hnd.oldHandPos(2)<<") Root : ("<<hnd.oldRootPos(0)<<", "<<hnd.oldRootPos(1)<<", "<<hnd.oldRootPos(2)<<") Shldr : ("<<hnd.oldrShldrPos(0)<<", "<<hnd.oldrShldrPos(1)<<", "<<hnd.oldrShldrPos(2)<<")"<<endl;
            // out<<"Drag sensitivity : "<<hnd.dragSens<<" Push In sens : "<<hnd.zSensMultIn<<" Push Out sens : "<<hnd.zSensMultOut<<endl;
            // out<<"Flag values : ";
            // for(int idx = 0; idx < KHnd_numFlags; ++idx){   out<<(idx == 0 ? " " : " | ")<<"IDX : "<<idx<<" VAL : "<<hnd.KHand_flags[idx];}
            // out<<endl;
            // out<<"Skel Space Tracking Box Multipliers : Low : ("<<hnd.hBxLowMlt[0]<<", "<<hnd.hBxLowMlt[1]<<", "<<hnd.hBxLowMlt[2]<<")  Hi : ("<<hnd.hBxHiMlt[0]<<", "<<hnd.hBxHiMlt[1]<<", "<<hnd.hBxHiMlt[2]<<")"<<endl;
            // out<<"Color : ("<<hnd.clr2d[0]<<", "<<hnd.clr2d[1]<<", "<<hnd.clr2d[2]<<", "<<hnd.clr2d[3]<<") Curr Image state : "<<hnd.imgState<<" Push Progress : "<<hnd.pushProgress<<endl;
            return out;
        }
    }//namespace kinect
}//namespace rtql8
