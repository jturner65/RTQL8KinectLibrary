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
#include "KinImageHandler.h"

using namespace std;

namespace rtql8{

	namespace kinect{

		KinImageHandler::KinImageHandler(INuiSensor* _oKin, KinectHandler* _owningKH) : 
			KinStreamHandler(_oKin, _owningKH, _H_RGB)
		{
			HRESULT tmpRes = initKinStreamHandler();
		}//cnstrctr
	
		HRESULT KinImageHandler::initKinStreamHandler(){
			HRESULT tmpRes = KinStreamHandler::initKinStreamHandler();		//base class
			initHndlrDataStructs();
			return tmpRes;
		}//initKinStreamHandler

		HRESULT KinImageHandler::initKinDataStream(){
			HRESULT imgStrOpenRes = owningKinect->NuiImageStreamOpen(
				rgbInitImgType,										// can also use NUI_IMAGE_TYPE_COLOR_INFRARED but needs to be either/or, can't have both at same time
				rgbInitRes,											// Image resolution
				rgbInitFlags,										// Image stream flags, 
				rgbInitFrameBuf,									// Number of frames to buffer - suggested in sdk
				m_hNextEvent,										// Event handle
				&KinStreamHandle);									//handle for rgb stream
			return imgStrOpenRes;
		}//initRGBStream

		void KinImageHandler::initHndlrDataStructs(){
			initVars();
		}//initHndlrDataStructs

		void KinImageHandler::initVars(){	
			STRM_Flags = vector<bool>(IM_NumFlags, false);
		}//initvars

				//build display image from data from kinect
		GLubyte* KinImageHandler::buildImage(GLubyte* rgbDataPtr){
			NUI_IMAGE_FRAME imgFrame;				//image metadata from kinect
			NUI_LOCKED_RECT LockedRect;				//ptr to data - locked so that it isn't corrupted upon reading
			if (owningKinect->NuiImageStreamGetNextFrame(KinStreamHandle, 0, &imgFrame) < 0) return rgbDataPtr;
			INuiFrameTexture* texture = imgFrame.pFrameTexture;							
			texture->LockRect(0, &LockedRect, NULL, 0);
			if (LockedRect.Pitch > 0)  {
				const BYTE* curr = (const BYTE*) LockedRect.pBits;
				const BYTE* dataEnd = curr + (strmImgW * strmImgH)*4;
				while (curr < dataEnd) {//bytes are in BGRA format
					for(int i = 0; i < 3; ++i){		
						rgbDataPtr[2-i] = *curr++;
					}	//bgra data - make last byte alpha val	
					*rgbDataPtr++;
					*rgbDataPtr++;
					*rgbDataPtr++;
					*rgbDataPtr++ = 0xff;										//opaque
					*curr++;
				}
			}//pitch = # bytes in each row - checking if not empty
			texture->UnlockRect(0);
			owningKinect->NuiImageStreamReleaseFrame(KinStreamHandle, &imgFrame);
			return rgbDataPtr;
		}//buildRGBImage


		//return appropriate kinect flag (or 0) for this stream corresponding to current setting of state machine flag 
		unsigned long KinImageHandler::getKHSMFlagMode(int idx, bool stream){//true->stream, false->frame
			bool val = owningKH->getFlag(idx);
			switch (idx){
				case _KH_NEAR_IDX			: {
					//return (val  ? (stream ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : NUI_IMAGE_FRAME_FLAG_NEAR_MODE_ENABLED ) : (unsigned long)0);}
					return (val  ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : (unsigned long)0);}
			}//switch
			return (unsigned long)0;
		}//getKHSMFlagMode

		bool KinImageHandler::setFlag(int idx, bool val){
			this->STRM_Flags[idx] = val;
			switch (idx){
				case _IM_SEATED_IDX		:{break;}
				case _IM_NEAR_IDX		:{break;}
				default : {break;}
			}//switch	
			return true;
		}//setFlag

	}//namespace kinect
}//namespace rtql8