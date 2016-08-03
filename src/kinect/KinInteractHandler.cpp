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

#include "KinInteractHandler.h"

using namespace std;
namespace rtql8{
	namespace kinect{

		KinInteractHandler::KinInteractHandler(INuiSensor* _oKin, KinectHandler* _owningKH): KinStreamHandler(_oKin, _owningKH, _H_INTERACT)		{
			HRESULT tmpRes = initKinStreamHandler();			//derived method
		}//KinInteractHandler

		HRESULT KinInteractHandler::initKinStreamHandler(){
			HRESULT tmpRes = KinStreamHandler::initKinStreamHandler();		//base class
			//any interaction stream specific initialisation here
			return tmpRes;
		}//initKinStreamHandler

		HRESULT KinInteractHandler::initKinDataStream(){
			HRESULT hr = NuiCreateInteractionStream(owningKinect, &intrClient, &intrStream);					//streams are inited
			if(FAILED(hr)){
				clog<<"\tKinInteractHandler : Failed to instantiate Interaction Stream code : "<<hr<<std::endl;
			}//handle stream failure																			//event handle for this stream

			m_hNextEvent = CreateEvent(NULL, TRUE, FALSE, NULL);												//need to rebuild event - use createevent instead of createeventw
			hr = intrStream->Enable(m_hNextEvent);
			if(FAILED(hr)){				
				clog<<"\tKinInteractHandler : Failed to instantiate Interaction Event : "<<hr<<std::endl;			
			}//handle event instance failure
			initHndlrDataStructs();																				//nothing stream-related
			return hr;
		}//initKinDataStream

		void KinInteractHandler::initHndlrDataStructs(){
			initVars();
		}//initHndlrDataStructs

		void KinInteractHandler::initVars(){
			STRM_Flags = vector<bool>(IH_NumFlags,false);
		}//initvars
	
			//build interaction stream to set hand location and status
		void KinInteractHandler::buildInteractData(){
			if((isSkelDpthProc()) && (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextEvent, 0))){												//checking event  
																														
				NUI_INTERACTION_FRAME Interaction_Frame;
				HRESULT hr = intrStream->GetNextFrame( 0,&Interaction_Frame );
				if (! SUCCEEDED(hr)){
					if(E_NUI_FRAME_NO_DATA == hr){	clog<<"\tKinInteractHandler : interaction failed code : E_NUI_FRAME_NO_DATA "<<hr<<std::endl;	}						// -2097086463
					else if (E_POINTER == hr){		clog<<"\tKinInteractHandler : interaction failed code : E_POINTER"<<hr<<std::endl;}
					else {							clog<<"\tKinInteractHandler : interaction failed code : unknown "<<hr<<std::endl;}						
					return;
				}		
				clog<<"\tKinInteractHandler : interaction success"<<std::endl;
				int trackingID = 0;
				int evnt=0;
				for(int i=0;i<NUI_SKELETON_COUNT;i++){
					trackingID = Interaction_Frame.UserInfos[i].SkeletonTrackingId; 
					 //   NUI_HAND_EVENT_TYPE_NONE = 0,   // No change from last event, or an undefined change.
						//NUI_HAND_EVENT_TYPE_GRIP,       // Hand grip
						//NUI_HAND_EVENT_TYPE_GRIPRELEASE // Hand grip release
  
					clog<<"\tKinInteractHandler : skel id="<<trackingID<<"  hand : "<<Interaction_Frame.UserInfos[i].HandPointerInfos[0].HandType<<" -----event:"<<Interaction_Frame.UserInfos[i].HandPointerInfos[0].HandEventType<<std::endl;
				}
					//clear proc flags to process another frame
				//IH_flags[IH_SkelProc] = false;
				//IH_flags[_IH_DPTH_PROC] = false;
				//ResetEvent(m_hNextEvent);
			}//if both skel and depth have been processed and event has been detected
		}//buildInteractData

			//skel frame needed by interactions to produce interaction frame		
		bool KinInteractHandler::ProcessSkeleton(NUI_SKELETON_FRAME &_skelFrame){
			HRESULT hr = 0;
			//if(!IH_flags[IH_SkelProc]) {
				Vector4 v;
				owningKinect->NuiAccelerometerGetCurrentReading(&v);
				hr = intrStream->ProcessSkeleton(NUI_NumSkels,_skelFrame.SkeletonData,&v,_skelFrame.liTimeStamp);
				STRM_Flags[_IH_SKEL_PROC] = (( FAILED( hr ) ) ? false : true);												//set flag depending on success/failure
				if(!STRM_Flags[_IH_SKEL_PROC]){clog<<"\tKinInteractHandler : Process Skeleton failed code : "<<hr<<std::endl;}
				//else{						clog<<"KinInteractHandler : Process of Skeleton successful"<<std::endl;}
			//}
			//clog<<"KinInteractHandler : Process Skeleton "<<(IH_flags[IH_SkelProc] ? "success" : "failed   : ")<<hr<<std::endl;
			return STRM_Flags[_IH_SKEL_PROC];
		}//setInteractSkelFrameData

			//depth frame needed by interactions to produce interaction frame
		bool KinInteractHandler::ProcessDepth(unsigned int &LRsize, unsigned char* &pbits , LARGE_INTEGER &timestamp){
			HRESULT hr = 0;
			//if(!IH_flags[IH_DpthProc]){																					//only process depth if none already processed
				hr = intrStream->ProcessDepth(LRsize, pbits, timestamp);
				STRM_Flags[_IH_DPTH_PROC] = (( FAILED( hr ) ) ? false : true);												//set flag depending on success/failure
				if(!STRM_Flags[_IH_DPTH_PROC]){	clog<<"\tKinInteractHandler : Process Depth failed code : "<<hr<<std::endl;}
				//else{						clog<<"KinInteractHandler : Process of Depth successful"<<std::endl;}
			//}
			//clog<<"KinInteractHandler : Process Depth "<<(IH_flags[IH_DpthProc] ? "success" : "failed   : ")<<hr<<std::endl;
			return STRM_Flags[_IH_DPTH_PROC];
		}//setInteractDepthFrameData

		unsigned long KinInteractHandler::getKHSMFlagMode(int idx, bool state){ return 0l;}
		bool KinInteractHandler::isSkelDpthProc(){return (STRM_Flags[_IH_DPTH_PROC] && STRM_Flags[_IH_SKEL_PROC]);}						//both the depth frame and the skeleton frame have been processed

		bool KinInteractHandler::setFlag(int idx, bool val){
			this->STRM_Flags[idx] = val;
			switch (idx){
				case _IH_SEATED_IDX		:{break;}
				case _IH_NEAR_IDX		:{break;}
				case _IH_SKEL_PROC		:{break;}
				case _IH_DPTH_PROC		:{break;}
				default : {break;}
			}//switch	
			return true;
		}//setFlag

	}//namespace kinect
}//namespace rtql8