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
#include "KinDepthHandler.h"
//#include <fstream>

using namespace std;

namespace rtql8{
    namespace kinect{

        KinDepthHandler::KinDepthHandler(INuiSensor* _oKin, KinectHandler* _owningKH): KinStreamHandler(_oKin,_owningKH, _H_DEPTH), 
                                                                                       handValAra(12),
                                                                                       LHandPressGrip(0), 
                                                                                       RHandPressGrip(0),
                                                                                       lHandMinMax(6),	rHandMinMax(6),
                                                                                       LLstRat(1), RLstRat(1), kNet()
        {
            HRESULT tmpRes = initKinStreamHandler();
        }//KinDepthHandler cnstrctr
	
        HRESULT KinDepthHandler::initKinDataStream(){
            HRESULT dpthStrmOpenRes = owningKinect->NuiImageStreamOpen(
                dpthInitImgType,																	// Depth camera or rgb camera?
                dpthInitRes,																		// depth Image resolution 
                getKHSMFlagMode(_KH_NEAR_IDX,true )| dpthInitFlags  ,									// depth stream flags, e.g. near mode
                dpthInitFrameBuf,																	// Number of frames to buffer
                m_hNextEvent,																		// Event handle
                &KinStreamHandle);																	// handle for depth stream
            return dpthStrmOpenRes;
        }//initDepthStream

        HRESULT KinDepthHandler::initKinStreamHandler(){
            HRESULT tmpRes = KinStreamHandler::initKinStreamHandler();						
            initHndlrDataStructs();
            return tmpRes;
        }//initKinStreamHandler

        void KinDepthHandler::initHndlrDataStructs(){
            initVars();
        }//initHndlrDataStructs

        void KinDepthHandler::initVars(){
            STRM_Flags = vector<bool>(DP_NumFlags, false);

            const BYTE val = 0x01;																	//initialize copy of valid data ara
            std::memset(lastValidDepthAra, val,strmImgW*strmImgH*4);
            LHandRng[2] = 80;						
            RHandRng[2] = 80;						
            LHandAra = nullptr;			//tmp array that will hold the left hand image - 4 bytes per pixel, 2*rng +1
            RHandAra = nullptr;			//tmp array that will hold the left hand image - 4 bytes per pixel, 2*rng +1
            LHandImgIdx = 0;
            RHandImgIdx = 0;
            LHandFillPxl = 0;
            RHandFillPxl = 0;
            lHandMinMax = vector<int>(6);						//idx xmin,xmax,ymin,ymax
            rHandMinMax = vector<int>(6);
        }//initVars

        HRESULT KinDepthHandler::setStreamEnableFlags(){
            HRESULT hr = owningKinect->NuiImageStreamSetImageFrameFlags( KinStreamHandle,  NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
            return hr;
        }//setStreamEnableFlags

        Eigen::VectorXi KinDepthHandler::getDepthRawHandDepthPos(){	return Eigen::VectorXi(handValAra);}

        //gets the locations in depthspace of the current raw hand positions from skel handler
        /**
           performing a linear regression of values of depth versus #of pxls comprising open hand image (~area of hand) yielded approx the following eq relating depth (x) to hand area (y): this will vary from hand to hand
           LHand : y = -6.16 x + 16500
           RHand : y = -4.85 x + 13200
        */
        bool KinDepthHandler::setHandLocs(int nearModeOn){
            if(0 == nearModeOn){HRESULT hr = owningKinect->NuiImageStreamSetImageFrameFlags( KinStreamHandle,  NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);	}//reenable nearmode
            handValAra = owningKH->getSkelRawHandDepthPos();											//call only 1 time from skel, call from depth (here) for all other calls
            if(-1 != handValAra[2]){																	//never have negative depth in legal depth value
                if(nullptr != RHandAra){delete[] RHandAra;}
                if(nullptr != LHandAra){delete[] LHandAra;}
                //left hand - map depth 400-4000 to LHandBx = array of vals for left hand track box = {max x, min x-max x, max y, min y-max y} 
                LHandRng[0] = (LHandBx[0] + int(LHandBx[1]*(handValAra[2] - 400)/(3600.0)));
                LHandRng[1] = (LHandBx[2] + int(LHandBx[3]*(handValAra[2] - 400)/(3600.0)));
                LHandAra = new GLubyte[(((2 * LHandRng[0]) +1) * ((2 * LHandRng[1])+1)) * 4];			//tmp array that will hold the left hand image - 4 bytes per pixel, 2*rng +1
                //right hand  RHandBx = array of vals for right hand track box = {max x, min x-max x, max y, min y-max y}
                RHandRng[0] = (RHandBx[0] + int(RHandBx[1]*(handValAra[5] - 400)/(3600.0)));
                RHandRng[1] = (RHandBx[2] + int(RHandBx[3]*(handValAra[5] - 400)/(3600.0)));			//joint not in center of hand, displace box up from joint
                RHandAra = new GLubyte[(((2 * RHandRng[0]) +1) * ((2 * RHandRng[1])+1)) * 4];			//tmp array that will hold the left hand image
                //idx xmin,xmax,ymin,ymax,dmin,dmax
                lHandMinMax[0]=1000,lHandMinMax[1]=-1000,lHandMinMax[2]=1000,lHandMinMax[3]=-1000,lHandMinMax[4]=1000,lHandMinMax[3]=-1000;	
                rHandMinMax[0]=1000,rHandMinMax[1]=-1000,rHandMinMax[2]=1000,rHandMinMax[3]=-1000,rHandMinMax[4]=1000,rHandMinMax[3]=-1000;
                LHandImgIdx = 0;																		//counts how many bytes we have put into the seg image ( divide by 4 for # pixels)
                RHandImgIdx = 0;
                LHandFillPxl = 0;																		//counts how many pixels of hand we've filled into segmentation image
                RHandFillPxl = 0;
                //boundaries for width and height of image strmImgW strmImgH
                boundXYLocsInDepthImg(handValAra[0],handValAra[1], true);
                boundXYLocsInDepthImg(handValAra[3],handValAra[4], false);
                return true;
            }
            return false;
        }//setHandLocs
        //sets minmax values for particular hand
        bool KinDepthHandler::setHandMinMax(bool left, int x, int y, int depthMM){
            if(left){
                lHandMinMax[0] = min(lHandMinMax[0],x);										//find extremal left hand vals : idx: xmin,xmax,ymin,ymax,depthmin,depthmax
                lHandMinMax[1] = max(lHandMinMax[1],x);
                lHandMinMax[2] = min(lHandMinMax[2],y);
                lHandMinMax[3] = max(lHandMinMax[3],y);
                lHandMinMax[4] = min(lHandMinMax[4],depthMM);
                lHandMinMax[5] = max(lHandMinMax[5],depthMM);
            } else {
                rHandMinMax[0] = min(rHandMinMax[0],x);										//find extremal right hand vals : idx: xmin,xmax,ymin,ymax
                rHandMinMax[1] = max(rHandMinMax[1],x);
                rHandMinMax[2] = min(rHandMinMax[2],y);
                rHandMinMax[3] = max(rHandMinMax[3],y);
                rHandMinMax[4] = min(rHandMinMax[4],depthMM);
                rHandMinMax[5] = max(rHandMinMax[5],depthMM);		
            }
            return true;
        }//setHandMinMax

        //determine whether right or left hand
        int KinDepthHandler::findHandType(int pxlIdx, USHORT depthMM){
            int retVal = 0, wyJD = 20, wzJD = 30;																	//wJD is distance from actual wrist joint to center of "hand box" in z - want to make box a little in front of joint because we are tracking hand and hand is facing cam, closer than joint
            int x = (pxlIdx % strmImgW), y = (pxlIdx / strmImgW);
            if((abs(x - handValAra[0]) <= LHandRng[0]) && (abs(y - handValAra[1]+wyJD) <= LHandRng[1])){			//build left hand array
                if (abs(depthMM - handValAra[2]+wzJD) < LHandRng[2]){	
                    setHandMinMax(true,x,y,depthMM);																//set extreme values for left hand - x,y, depth
                    if(this->STRM_Flags[_DP_BUILD_HNDIMG]){										
                        LHandAra[LHandImgIdx++] = (byte)0x00;	LHandAra[LHandImgIdx++] = (byte)0xff;	LHandAra[LHandImgIdx++] = (byte)0x00;	LHandAra[LHandImgIdx++] = (byte)0xff;		//this builds isolated left hand image
                    }
                    LHandFillPxl++;	retVal = 1;	} //left hand
                else {	
                    if(this->STRM_Flags[_DP_BUILD_HNDIMG]){									
                        LHandAra[LHandImgIdx++] = (byte)0x00;	LHandAra[LHandImgIdx++] = (byte)0x00;	LHandAra[LHandImgIdx++] = (byte)0x00;	LHandAra[LHandImgIdx++] = (byte)0x00;
                    }//build hnd img
                }//if pxl at right dpth
            }//if left hand
            else if((abs(x - handValAra[3]) <= RHandRng[0]) && (abs(y - handValAra[4]+wyJD) <= RHandRng[1])){
                if (abs(depthMM - handValAra[5]+wzJD) < RHandRng[2]){			
                    setHandMinMax(false,x,y,depthMM);
                    if(this->STRM_Flags[_DP_BUILD_HNDIMG]){										
                        RHandAra[RHandImgIdx++] = (byte)0xff;	RHandAra[RHandImgIdx++] = (byte)0x00;	RHandAra[RHandImgIdx++] = (byte)0x00;	RHandAra[RHandImgIdx++] = (byte)0xff;		//this builds isolated right hand image
                    }
                    RHandFillPxl++;	retVal = 2;	}//right hand
                else {	
                    if(this->STRM_Flags[_DP_BUILD_HNDIMG]){										
                        RHandAra[RHandImgIdx++] = (byte)0x00;	RHandAra[RHandImgIdx++] = (byte)0x00;	RHandAra[RHandImgIdx++] = (byte)0x00;	RHandAra[RHandImgIdx++] = (byte)0x00;
                    }//build hnd img
                }//if pxl at right depth
            }//if right hand
            return retVal;
        }//findHandType

        GLubyte* KinDepthHandler::getDepthHandImg(bool isLeft){		return (isLeft ? LHandAra : RHandAra);	}
        int KinDepthHandler::getHandImageWidth(bool isLeft){		return (isLeft ? ((2 * LHandRng[0]) +1) : ((2 * RHandRng[0]) +1));		}
        int KinDepthHandler::getHandImageHeight(bool isLeft){		return (isLeft ? ((2 * LHandRng[1]) +1) : ((2 * RHandRng[1]) +1));		}

        //std::ofstream fout2("log2.txt");

        //build display image based on depth data
        GLubyte* KinDepthHandler::buildImage(GLubyte* depthDataPtr){												//.4m - 3m = 400-3000 mm, .8 - 4 m for non-near mode
            if (0 > owningKinect->NuiImageStreamGetNextFrame(KinStreamHandle, 2, &imgFrame)) return depthDataPtr;	//get depth frame	
            //int minDepthVal = 1000000, maxDepthVal = 0;
            NUI_LOCKED_RECT LockedRect;																				//ptr to data - locked so that it isn't corrupted upon reading
            INuiFrameTexture* texture;	
            BOOL nearMode;																							//populated by NuiImageFrameGetDepthImagePixelFrameTexture - used by interactions
            if (0 > owningKinect->NuiImageFrameGetDepthImagePixelFrameTexture(KinStreamHandle, &imgFrame, &nearMode, &texture)) return depthDataPtr;	//need to use this for interactions
            texture->LockRect(0, &LockedRect, NULL, 0);																//lock texture so it doesn't change as we read it
            bool handsFound = false;
            if (LockedRect.Pitch > 0)  {
                if(this->STRM_Flags[_DP_PROC_HNDS]){ handsFound =	setHandLocs(nearMode);	}				        //finds hands in depth image if processing hand data
                unsigned int LRsize = LockedRect.size;
                int pxlIdx = 0;
                owningKH->ProcessDepth(LRsize, (LockedRect.pBits), imgFrame.liTimeStamp);							//for interaction client
                //pixels in lockedrect are in NUI_DEPTH_IMAGE_PIXEL format
                const NUI_DEPTH_IMAGE_PIXEL* curr = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL*>(LockedRect.pBits);
                const NUI_DEPTH_IMAGE_PIXEL* dataEnd = curr + (strmImgW*strmImgH);
                BYTE* validDpthData = lastValidDepthAra;													
                BYTE* savedValidDpthData = validDpthData;

                while (curr < dataEnd) {
                    USHORT depthMM = curr->depth;
                    int handType = (handsFound ? findHandType(pxlIdx, depthMM) : 0);	  //not a hand = 0, L = 1, R = 2

                    if(validDepth(depthMM)){		//valid depth value - may be too close or too far									
                        //if(depthMM <= 399){			for(int i = 0; i < 4; ++i){	*depthDataPtr++ = 0xff;	}}									//very close - white
                        //else if(depthMM >= 4000){	for(int i = 0; i < 4; ++i){	*depthDataPtr++ = 0x00;	}}// *depthDataPtr++ = 0xff;}			//very far - trans
                        //else 
                        //{//in range 400-4000
                        if (0 == handType){
                            BYTE depthVal = (BYTE)((depthMM-400)/14);
                            depthVal = (((depthVal <= (BYTE)0xff) && (depthVal >= 0)) ? depthVal : ((depthVal > (BYTE)0xff) ? (BYTE)0xff : 0));
                            for(int i = 0; i < 3; ++i){	
                                *depthDataPtr++ = (BYTE)0xff - depthVal;	*validDpthData++ =  (BYTE)0xff - depthVal;		
                            }//lighter - closer, darker - further
                            *depthDataPtr++ = (BYTE)0xff;		*validDpthData++ = (BYTE)0xff;												
                        }
                        else if((1 == handType) && (this->STRM_Flags[_DP_DISP_HNDS])){						//left - green
                            *depthDataPtr++ =  (BYTE)0x00;	*depthDataPtr++ =  (BYTE)0xff;	*depthDataPtr++ =  (BYTE)0x00;	*depthDataPtr++ =  (BYTE)0xff;	
                            *validDpthData++ = (BYTE)0x00;  *validDpthData++ = (BYTE)0xff;	*validDpthData++ = (BYTE)0x00;  *validDpthData++ = (BYTE)0xff;
                        } 
                        else if((2 == handType) && (this->STRM_Flags[_DP_DISP_HNDS])){						//right - red
                            *depthDataPtr++ =  (BYTE)0xff;  *depthDataPtr++ =  (BYTE)0x00;	*depthDataPtr++ =  (BYTE)0x00;   *depthDataPtr++ = (BYTE)0xff;	
                            *validDpthData++ = (BYTE)0xff;	*validDpthData++ = (BYTE)0x00;  *validDpthData++ = (BYTE)0x00;	*validDpthData++ = (BYTE)0xff;												
                        }
                        //}//valid dpth pxl value
                    } else {
                        switch(curr->depth){//keep  *validDpthData up to data - maybe use the values pointed at by this pointer to populate depth if depth not known
                        case 0x0000 :{//too near - .8m to .4m - only visible if not in near mode - make white
                            for(int i = 0; i < 4; ++i){	*depthDataPtr++ = 0xff; *validDpthData++;	}
                            break;}
                        case 0x7ff8 :{//too far - 4m to 8m (3m to 8m if in near mode make transparent blue
                            for(int i = 0; i < 2; ++i){	*depthDataPtr++ = 0x00;	*validDpthData++;	}
                            *depthDataPtr++ = 0xff;	*validDpthData++;*depthDataPtr++ = 0xff;	*validDpthData++;
                            break;}
                        case 0xfff8 :{//unknown - less than .4m, greater than 8m
                            *depthDataPtr++ = 0xff;*validDpthData++;
                            for(int i = 0; i <2; ++i){	*depthDataPtr++ = 0x00;	*validDpthData++;}
                            *depthDataPtr++ = 0xff;*validDpthData++;
                            break;}
                        }//switch
                    }//if valid else
                    *curr++;
                    pxlIdx++;
                }//while pxls

            }//pitch = # bytes in each row - checking if not empty
            else {				
                clog<<"\tKinDepthHandler : No depth image retrieved "<<std::endl;
            }
			if(handsFound){          setPressOrGrip();    }//if hands found

			texture->UnlockRect(0);
			owningKinect->NuiImageStreamReleaseFrame(KinStreamHandle, &imgFrame);									//may not want to do this since we use this frame for interaction handler
			return depthDataPtr;


            //    fout2 << endl;
            //    fout2 << "Thresholds : " << DP_GRIPW_THRESH << " " << DP_GRIPH_THRESH << " " << DP_GRIPD_THRESH << endl;
            //    fout2 << "lHandMinMax : " << lHandMinMax[0] << " " << lHandMinMax[1] << " " << lHandMinMax[2];
            //    fout2 << " " << lHandMinMax[3] << " " << lHandMinMax[4] << " " << lHandMinMax[5] << endl;
            //    fout2 << "LhandRng : " << LHandRng[0] << " " << LHandRng[1] << " " << LHandRng[2] << endl;
        }//buildDepthImage

        unsigned long KinDepthHandler::getKHSMFlagMode(int idx, bool stream){//true->stream, false->frame
            //return appropriate kinect flag (or 0) for this stream corresponding to current setting of state machine flag 
            bool val = owningKH->getFlag(idx);
            switch (idx){
            case _KH_NEAR_IDX			: {
                //return (val  ? (stream ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : NUI_IMAGE_FRAME_FLAG_NEAR_MODE_ENABLED ) : (unsigned long)0);}
                return (val  ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : (unsigned long)0);}
            }//switch
            return (unsigned long)0;
        }//getKHSMFlagMode

        //              min x                    max x              min y               max y           min depth           max depth           bnd box x      bnd box y        bnd box d           depth of joint
        //_inVals : lHandMinMax[0]<<","<<lHandMinMax[1]<<","<<lHandMinMax[2]<<","<<lHandMinMax[3]<<lHandMinMax[4]<<","<<lHandMinMax[5]<<","<<LHandRng[0]<<","<<LHandRng[1]<<","<<LHandRng[2]<<","<<handValAra[depthidx = 2(left) || 5(right) (joint depth)
        void KinDepthHandler::setPressOrGrip(){
            if((STRM_Flags[_DP_USE_NN]) && (kNet.isBuilt())){                             // use nn to classify hand state - TODO :verify kNet exists
                vector<int> _LinVals(9), _RinVals(9);
                for(int i = 0; i < lHandMinMax.size(); ++i){     _LinVals.push_back(lHandMinMax[i]);                 _RinVals.push_back(rHandMinMax[i]);                  }
                for(int i = 0; i < 3; ++i){                      _LinVals.push_back(LHandRng[i]);                   _RinVals.push_back(RHandRng[i]);                  }

                kNet.feedForward(_LinVals);
                LHandPressGrip = (1 == kNet.getOutputAsVector()[0] ? 0 : 2 );                                   //classify left hand

                kNet.feedForward(_RinVals);
                RHandPressGrip = (1 == kNet.getOutputAsVector()[0] ? 0 : 2 );                                   //classify right hand

            } else {
                if(STRM_Flags[_DP_RECORD_NN]){//record data for NNet training/testing - swap min and max x's for right and left hand, to double train/test data
                    recordHands();                    
                } else {
				    float 
				    //LRatV = LHandFillPxl/(1.0 *(((2 * LHandRng[0]) +1) * ((2 * LHandRng[1])+1)) ), RRatV = RHandFillPxl/(1.0 *(((2 * RHandRng[0]) +1) * ((2 * RHandRng[1])+1)) ),					//ratio of filled pxls vs area of entire hand square
				    LRatD = (lHandMinMax[5] - lHandMinMax[4])/(1.0*LHandRng[2]),	RRatD = (rHandMinMax[5] - rHandMinMax[4])/(1.0*RHandRng[2]),
				    LRatH = (lHandMinMax[3] - lHandMinMax[2])/(1.0*LHandRng[1]),	RRatH = (rHandMinMax[3] - rHandMinMax[2])/(1.0*RHandRng[1]),
				    LRatW = (lHandMinMax[1] - lHandMinMax[0])/(1.0*LHandRng[0]),	RRatW = (rHandMinMax[1] - rHandMinMax[0])/(1.0*RHandRng[0]);
					//ratio of height to width of hand img - >DP_GRIP_THRESH is probably open, <DP_GRIP_THRESH is probably grip
				    if((LRatH > 0) && (LRatW > 0) && (LRatD > 0) && ((LRatH < DP_GRIPH_THRESH) || (LRatD < DP_GRIPD_THRESH) || (LRatW < DP_GRIPW_THRESH))){
                        LHandPressGrip = 2;																				//grip
					    if((LRatD >= .75f * DP_GRIPD_THRESH) && (LRatH <  .75f * DP_GRIPH_THRESH) && (LRatW > DP_GRIPW_THRESH)){	LHandPressGrip = 1;}		//press		not that reliable, and noisy use movement in z to determine press/click at least for 2d ui	
				    } else {					LHandPressGrip = 0;}

				    if((RRatH > 0) && (RRatW > 0) && (RRatD > 0) && ((RRatH < DP_GRIPH_THRESH) || (RRatD < DP_GRIPD_THRESH) || (RRatW < DP_GRIPW_THRESH))){
					    RHandPressGrip = 2;																				//grip
					    if((RRatD >= .75f * DP_GRIPD_THRESH) && (RRatH <  .75f * DP_GRIPH_THRESH) && (RRatW > DP_GRIPW_THRESH)){	RHandPressGrip = 1;}
				    } else {					RHandPressGrip = 0;}
                }//if
            }//if use NN else
        }//setPressOrGrip

        bool KinDepthHandler::setFlag(int idx, bool val){
            this->STRM_Flags[idx] = val;
            switch (idx){
            case _DP_SEATED_IDX		:{break;}
            case _DP_NEAR_IDX		:{break;}
            case _DP_PROC_HNDS		:{//process hand location/grip state
                if(!val)	{		//turn off hand img proc and disp
                    STRM_Flags[_DP_DISP_HNDS] = false;
                    STRM_Flags[_DP_PROC_HNDS] = false;
                    LHandPressGrip = 0;			//unclench!
                    RHandPressGrip = 0;
                }
                break;}		
            case _DP_DISP_HNDS		:{//display hands in depth image
                if((val) && (!STRM_Flags[_DP_PROC_HNDS])){		STRM_Flags[_DP_PROC_HNDS] = true;}
                break;}						
            case _DP_BUILD_HNDIMG	:{//build hand images from processed depth data
                if((val) && (!STRM_Flags[_DP_PROC_HNDS])){		STRM_Flags[_DP_PROC_HNDS] = true;}
                break;}		
            case _DP_RECORD_NN :{   
                break;}
            case _DP_USE_NN    :{   
                break;}
            default : {break;}
            }//switch	
            return true;
        }//setFlag
    }//namespace kinect
}//namespace rtql8


// Sehoon's auxiliary variables
//int pixelX = (pxlIdx % strmImgW);
//int pixelY = (pxlIdx / strmImgW);
//handvector.push_back(handType);

// Sehoon's codes
//if (cntLeft > 0) {
//    const int TYPE_NOT_HAND = 0;
//    const int TYPE_LEFT     = 1;
//    std::vector<int> hands;
//    std::vector<int> silhouettes;
//    int dnum = 8;
//    int dx[10] = {-1, -1, -1,  0, 0,  1, 1, 1, 0, 0};
//    int dy[10] = {-1,  0,  1, -1, 1, -1, 0, 1, 0, 0};
//    for (int i = 0; i < handvector.size(); i++) {
//        int px = (i % strmImgW);
//        int py = (i / strmImgW);
//        int t = handvector[i];
//        if (t != TYPE_LEFT) {
//            continue;
//        }
//        hands.push_back(i);
//        bool is_silhouettes = false;
//        for (int j = 0; j < dnum; j++) {
//            int xx = px + dx[j];
//            int yy = py + dy[j];
//            if (xx < 0 || yy < 0 || xx >= strmImgW || yy >= strmImgH) {
//                continue;
//            }
//            int tt = handvector[xx + yy * strmImgW];
//            if (tt != TYPE_NOT_HAND) {
//                continue;
//            }
//            is_silhouettes = true;
//            break;
//            break;
//        }
//        if (is_silhouettes) {
//            silhouettes.push_back(i);
//            mark(savedDepthDataPtr, savedValidDpthData, px, py,
//                 0x00, 0x00, 0xff, 0xff, 1);

//        }
//    }

//    int max_radius = 0;
//    int cx = 0, cy = 0;
//    for (int i = 0; i < hands.size(); i++) {
//        double min_dist = 9999;
//        int xi= (hands[i] % strmImgW);
//        int yi = (hands[i] / strmImgW);
//        
//        for (int j = 0; j < silhouettes.size(); j++) {
//            int xj= (silhouettes[j] % strmImgW);
//            int yj = (silhouettes[j] / strmImgW);
//            double dx = (double)(xi - xj);
//            double dy = (double)(yi - yj);
//            double dist = sqrt(dx * dx + dy * dy);
//            if (dist < min_dist) {
//                min_dist = dist;
//            }
//        }
//        if (max_radius < min_dist) {
//            max_radius = min_dist;
//            cx = xi;
//            cy = yi;
//        }
//    }
//    int avgLeftX = cx;
//    int avgLeftY = cy;
//    // clog << "pt = " << cx << " " << cy << " ";
//    // clog << "max_radius = " << max_radius << endl;
//    // int avgLeftX = (int)sumLeftX / cntLeft;
//    // int avgLeftY = (int)sumLeftY / cntLeft;
//    mark(savedDepthDataPtr, savedValidDpthData, avgLeftX, avgLeftY,
//         0xff, 0x00, 0xff, 0xff);
//    // clog << "avg = " << avgLeftX << " " << avgLeftY << endl;
//    // int avgPixelID = avgLeftY * strmImgW + avgLeftX;
//    // const int BYTE_PER_PIXEL = 4;
//    // int depthPixelID = avgPixelID * BYTE_PER_PIXEL; 
//    // avgLeftX = 100;
//    // avgLeftY = 130;
//    

//    double minDistLeft = 9999;
//    double maxDistLeft = -9999;
//    int x0, y0;
//    int x1, y1;

//    for (int i = 0; i < silhouettes.size(); i++) {
//        int px = (silhouettes[i] % strmImgW);
//        int py = (silhouettes[i] / strmImgW);
//        if (py > avgLeftY) {
//            continue;
//        }
//        double dx = fabs((double)(px - avgLeftX));
//        double dy = fabs((double)(py - avgLeftY));
//        if (dx > 5.0 || dy < 10.0) {
//            continue;
//        }

//        double d = sqrt(dx * dx + dy * dy);
//        if (d < minDistLeft) {
//            x0 = px;
//            y0 = py;
//            minDistLeft = d;
//        }
//        if (d > maxDistLeft) {
//            x1 = px;
//            y1 = py;
//            maxDistLeft = d;
//        }
//    }
//    double ratio = maxDistLeft / minDistLeft;
//    // clog << "ratio = " << ratio << "(" << maxDistLeft << "/" << minDistLeft << ")";
//    // if (ratio < 1.1) {
//    // if (ratio < 1.2) {
//    //     clog << "!!!!!!!!!!!!!!! ";
//    //     clog << "GRAB!!!!";
//    //     clog << "!!!!!!!!!!!!!!! ";
//    //     clog << endl;
//    //     LHandPressGrip = 2;
//    // } else {
//    //     LHandPressGrip = 0;
//    // }
//    // clog << endl;
//    // clog << "avg = " << avgLeftX << " " << avgLeftY << ",   ";
//    // clog << "p0 = " << x0 << " " << y0 << endl;
//    mark(savedDepthDataPtr, savedValidDpthData, x0, y0,
//         0xff, 0x00, 0xff, 0xff);
//    mark(savedDepthDataPtr, savedValidDpthData, x1, y1,
//         0xff, 0x00, 0xff, 0xff);
//    // LHandPressGrip = 2;
//}
    

//void mark(GLubyte* depthDataPtr, BYTE* validDpthData, int x, int y, BYTE R, BYTE G, BYTE B, BYTE D, int SIZE = 8) {
//    for (int i = x - SIZE; i <= x + SIZE; i++) {
//        for (int j = y - SIZE ; j <= y + SIZE; j++) {
//            const int BYTE_PER_PIXEL = 4;
//            int avgPixelID = j * strmImgW + i;
//            if (avgPixelID < 0) continue;
//            if (avgPixelID > strmImgW * strmImgH) continue;
//            int depthPixelID = avgPixelID * BYTE_PER_PIXEL; 
//            *(depthDataPtr + depthPixelID + 0) = R;
//            *(depthDataPtr + depthPixelID + 1) = G;
//            *(depthDataPtr + depthPixelID + 2) = B;
//            *(depthDataPtr + depthPixelID + 3) = D;

//            *(validDpthData + depthPixelID + 0) = R;
//            *(validDpthData + depthPixelID + 1) = G;
//            *(validDpthData + depthPixelID + 2) = B;
//            *(validDpthData + depthPixelID + 3) = D;
//        }
//    }  
//}//mark