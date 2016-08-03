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



#ifndef __KININTERACTHANDLER_H__
#define __KININTERACTHANDLER_H__


#include "KinStreamHandler.h"
#include "KinInteractClient.h"

using namespace std;
//all functionality to process interactive stream data from kinect
namespace rtql8 {
	namespace kinect{
		class KinectHandler;
		class KinStreamHandler;

		class KinInteractHandler : public KinStreamHandler{
		public:
			KinInteractHandler(INuiSensor* _oKin, KinectHandler* _owningKH);
			~KinInteractHandler(void){
				if(nullptr != intrStream){		intrStream->Release();	}
			}//dstrctr
			
			HRESULT initKinStreamHandler();
			HRESULT initKinDataStream();
			void initHndlrDataStructs();	

			bool ProcessSkeleton(NUI_SKELETON_FRAME& _skelFrame);
			bool ProcessDepth(unsigned int &, unsigned char* &, LARGE_INTEGER &);

			void buildInteractData();

			bool isSkelDpthProc();																				//both the depth frame and the skeleton frame have been processed
			
			unsigned long getKHSMFlagMode(int idx, bool state);
			virtual bool setFlag(int idx, bool val);


		private : //methods
			void initVars();
		
		public : //variables
			KinInteractClient			intrClient;
			INuiInteractionStream*		intrStream;																//handle to interaction stream - do not use Streamhandler's stream, this one needs to inherit INuiInteractionStream
		};
	}//namespace kinect
}//namespace rtql8

#endif
