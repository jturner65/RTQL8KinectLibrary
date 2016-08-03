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

#ifndef __KINCNTRLHAND_H__
#define __KINCNTRLHAND_H__

#include "KinHndlrConsts.h"
#include "KinectController.h"


namespace rtql8{
    namespace kinect{
        class KinectController;
        class KinCntrlHand{
        public: 
            KinCntrlHand(KinectController* _kc, int _hnd);
            ~KinCntrlHand(){}

			void initHand(){				                                                //set initial hand positions    
                //2d ui values
                gHandLoc = Eigen::Vector3i(initLoc[0],initLoc[1],0);	
                oldgHandLoc = Eigen::Vector3i(gHandLoc);
                newScnHandLoc = Eigen::Vector3i(gHandLoc);
                oldScnHandLoc = Eigen::Vector3i(newScnHandLoc);
                //3d ui values - in skel space
                spcHandLoc = Eigen::Vector3d((KC_LEFT == hand ? -1 : 1),1,0);                                      

                //values from streams
				handLocPos = Eigen::Vector3d(initLoc[0],initLoc[1],0);                      //default locations of hands on screen for 2d display
                rootLocPos = Eigen::Vector3d(0,0,0);
                rShldrLocPos = Eigen::Vector3d(0,0,0);
                oldHandPos = Eigen::Vector3d(handLocPos);
                oldRootPos = Eigen::Vector3d(0,0,0);
                oldrShldrPos = Eigen::Vector3d(0,0,0);
            }//inithands

            //draw routines - from KinectController
            void draw2d();
            void draw3d();

            //hand location handlers - from kinect controller
            void handleHand(Eigen::VectorXd& handValLocAra, int gripState);
            void setNewHandVals2d(Eigen::VectorXd& handValLocAra, int gripState);
            void calc2dHandMapping();
            void calc2dPush();
            bool chkStateChange(){	return (handState != oldHandState);	}
            bool chkStClkUp(){ return ((0 == handState) && (0 != oldHandState));}                        //transition from click/grip to release
            bool chkStClkDown(){ return ((0 != handState) && (0 == oldHandState));}                      //transition from open to click
            void setOldHandVals();
            void drawHndVelVec(bool left);
            
            bool getFlag(int idx){ return KHand_flags[idx];}

            int getHandState(){return handState;}
            int getOldHandState(){return oldHandState;}

            //set sticky hand location from event handler
            void setNewHandLoc2d(int _x, int _y){  gHandLoc(0) = _x; gHandLoc(1) = _y;}
            //set whether this hand is dragging an object or not
            void setFlags(int idx, bool _val);
            //set initial hand location - need to have a window instantiated before calling this
            void setInitLoc(int _x, int _y){    
                initLoc[0] = _x; initLoc[1] = _y;
                initHand();                                                         //reinitialize hands
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            friend std::ostream& operator<<(std::ostream& out, const KinCntrlHand& hnd);

        private: //functions
            void drawHand2d();
            void drawHand3d();
            void drawPushProgress();
            void boundHandLoc();
            int mapSkelSpcToScrn(double x0, double x1, double y0, double y1, double xVal){	        //map skeleton space coord to screen coords
                if(xVal >= x1){return int(y1);}                                                 //bound to max y
                else if (xVal <= x0){return int(y0);}                                           //bound to min y
                return int(y0 + ((xVal - x0)/(x1 - x0)) * (y1 - y0));				
            }//mapSkelToScrn

        public : //variables

        private : 
            KinectController* kc;                                                   //owning Kinect Controller

                //representation
            int hand;                                                               //which hand is this
            vector<int> initLoc;                                                    //initial/untracked location for hands
            vector<float> clr2d;                                                    //the color of this hand in 2d
            vector<float> dim2d;                                                    //base dimension multiplier for hand drawing in d2 - controls overall size of hand

                //events
            vector<bool> KHand_flags;                                               //state flags for this hand
            int handState;														    //current state of each hand - open, click, grab
            int oldHandState;													    //previous state of each hand - open, click, grab						
            int imgState;                                                           //what image the hand should use - 0 - base, 1 - press, 2 - fist
            float pushProgress;                                                     //how much progress toward a push transition
            int lastObjID;                                                          //last object interacted with - -1 for none

                //tracking and sensitivity
            vector<double> hBxLowMlt, hBxHiMlt;                                      //multipliers used to build tracking box to translate from skel space to screen space
            double zSensMultIn, zSensMultOut;                                        //sensitivity multipliers for push progress in and out, from globals in KinectController
            double dragSens;                                                         //sensitivity during dragging

                //location for 2D processing
            Eigen::Vector3i gHandLoc, oldgHandLoc;                                  //current and old screen location of hand
            Eigen::Vector3i newScnHandLoc, oldScnHandLoc;                           //current and old translation from skel space to screen space of hand                                    
                //3D gui/avatar space
            Eigen::Vector3d spcHandLoc;                                             //current translation for hand in avatar space (3d)
                //from handlers
			Eigen::Vector3d handLocPos, rootLocPos, rShldrLocPos;					//current locations in skeleton space of hand, root, shoulder
			Eigen::Vector3d oldHandPos, oldRootPos, oldrShldrPos;	                //previous hand position in depth space of hand, root, shoulder

        };//class KinCntrlHand

    }//namespace kinect
}//namespace rtql8
#endif
