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

#ifndef __KINDEPTHHANDLER_H__
#define __KINDEPTHHANDLER_H__

#include "KinStreamHandler.h"
#include "KinNNet.h"

using namespace std;
//all functionality to process depth data from kinect
namespace rtql8{
    namespace kinect{
        class KinectHandler;
        class KinStreamHandler;
        class KinDepthHandler : public KinStreamHandler{
        public:
            KinDepthHandler(INuiSensor* _oKin, KinectHandler* _owningKH);
            ~KinDepthHandler(void){
                if (nullptr != RHandAra) delete[] RHandAra;
                if (nullptr != LHandAra) delete[] LHandAra;
            }

            HRESULT initKinStreamHandler();	
            HRESULT initKinDataStream();																					//overrides base class method
            void initHndlrDataStructs();

            bool setHandMinMax(bool left, int x, int y, int depth);

        private : //methods
            void initVars();
            bool setHandLocs(int nearMode);
            void boundXYLocsInDepthImg(int& x, int& y, bool lHand){
                int bx = (lHand ? LHandRng[0] : RHandRng[0]);
                int by = (lHand ? LHandRng[1] : RHandRng[1]);
                x = ((bx < x) && (strmImgW-bx > x) ? x : ((bx > x) ? bx : strmImgW - bx));
                y = ((by < y) && (strmImgH-by > y) ? y : ((by > y) ? by : strmImgH - by));
            }//boundXYLocsInDepthImg

            int findHandType(int idx, USHORT depthMM);
	
        public : //methods
            GLubyte* buildImage(GLubyte* depthDataPtr);
            GLubyte* getDepthHandImg(bool isLeft);
            int getHandImageWidth(bool isLeft);
            int getHandImageHeight(bool isLeft);
            //GLuint getHandImgTextureID(bool isLeft);(left ? this->LHandImgID : this->RHandImgID)
            int getPressGripState(){		return (LHandPressGrip  + (RHandPressGrip * 10));}								//place 0 is left hand, 1 is right hand

            bool validDepth(USHORT _dpth){ return ((_dpth != 0x0000) && (_dpth != 0x7ff8) && (_dpth != 0xfff8));}	//sentinel dpth values for too short, too long and unknown
            HRESULT setStreamEnableFlags();														                    //overrides base class method
            unsigned long getKHSMFlagMode(int idx, bool state);
            Eigen::VectorXi getDepthRawHandDepthPos();

            virtual bool setFlag(int idx, bool val);

            void setPressOrGrip();

            void startHandRec(string fname, int _HandState){
                setFlag(_DP_RECORD_NN, true);
                curLearnHandState = _HandState;
                handOut.open(fname.c_str(), std::ofstream::out | std::ofstream::app);  
            }//startHandRec

            void stopHandRec(){
                this->setFlag(_DP_RECORD_NN, false);
                handOut.close();   
            }

            void setKinNNet(KinNNet& _kn){kNet = _kn;}

            //record current hand positions and dimensions
            //currently supports 2 hand states - 0 : open, 2 : grip
            void recordHands(){
                if(handOut.is_open()){
                    handOut<<lHandMinMax[0]<<","<<lHandMinMax[1]<<","<<lHandMinMax[2]<<","<<lHandMinMax[3]<<lHandMinMax[4]<<","<<lHandMinMax[5]<<","<<LHandRng[0]<<","<<LHandRng[1]<<","<<LHandRng[2]<<","<<handValAra[2]<<","<<curLearnHandState<<std::endl;
                    handOut<<rHandMinMax[1]<<","<<rHandMinMax[0]<<","<<rHandMinMax[2]<<","<<rHandMinMax[3]<<rHandMinMax[4]<<","<<rHandMinMax[5]<<","<<RHandRng[0]<<","<<RHandRng[1]<<","<<RHandRng[2]<<","<<handValAra[5]<<","<<curLearnHandState<<std::endl;
                    handOut<<rHandMinMax[0]<<","<<rHandMinMax[1]<<","<<rHandMinMax[2]<<","<<rHandMinMax[3]<<rHandMinMax[4]<<","<<rHandMinMax[5]<<","<<RHandRng[0]<<","<<RHandRng[1]<<","<<RHandRng[2]<<","<<handValAra[5]<<","<<curLearnHandState<<std::endl;
                    handOut<<lHandMinMax[1]<<","<<lHandMinMax[0]<<","<<lHandMinMax[2]<<","<<lHandMinMax[3]<<lHandMinMax[4]<<","<<lHandMinMax[5]<<","<<LHandRng[0]<<","<<LHandRng[1]<<","<<LHandRng[2]<<","<<handValAra[2]<<","<<curLearnHandState<<std::endl;
                } else {
                    clog<<"\tKinDepthHandler : ERROR : No Hand file open to record hand locations "<<endl;
                }
            }//recordHands

            
        public : //variables
            GLubyte lastValidDepthAra[strmImgW*strmImgH*4];									// ara contains most recent valid data for depth - used to fill in invalid data locations
            NUI_IMAGE_FRAME imgFrame;														//image metadata - built in buildImage - used for interactions

        private : //variables

            Eigen::VectorXi handValAra;					                            	//array from skel handler holding curr hand locations in depth space (dpth space loc calced in skel handler) - l(x,y,d) : idx 0,1,2 ; r(x,y,d) : idx 3,4,5																						

            std::ofstream handOut;                                                      //file to write hand data to

            int LHandRng[3]; 
            int RHandRng[3];							                            	//boundary around hand joints to catch entire hand within this box we will look for depth pixels and assume any we see belong to that hand 
            int curLearnHandState;                                                      //current hand state being recorded - 0 is open, 2 is grip
            GLubyte* LHandAra; 
            GLubyte* RHandAra;							                            	// byte array holding pixels(clusters of 4 bytes) of segmentation image for each hand

            int LHandPressGrip;							                            	//1 for press, 2 for grip
            int RHandPressGrip;							                            	//whether or not a hand is gripping

            int LHandImgIdx;					
            int RHandImgIdx;							                            	//current byte index in handimgara - subtract 1 and divide by 4 to get # of bytes

            int LHandFillPxl; 
            int RHandFillPxl;							                            	//total # of hand pxls.
            float LLstRat, RLstRat;						                            	//last reasonable ratio of height to width of tracked hand

            vector<int> lHandMinMax;
            vector<int> rHandMinMax;

            KinNNet kNet;                                                               //neural net to decipher hand grab/push/nothing actions
        };
    }//namespace kinect
}//namespace rtql8
#endif
