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
#include "KinectHandler.h"
#include "KinStreamHandler.h"
#include "KinImageHandler.h"
#include "KinDepthHandler.h"
#include "KinSkelHandler.h"
#include "KinInteractHandler.h"
#include "KinAudioHandler.h"

using namespace std;

namespace rtql8{
	namespace kinect{

		KinectHandler::KinectHandler(): kinectSnsr(nullptr), _tmpSkelIKHandles(NUI_NumJnts), _tmpSkelIKHandleNames(NUI_NumJnts),
										flags(KH_NumFlags, false), reConnectCountDown(_RCD), camAngle(0),
										imgHandler(nullptr), dpthHandler(nullptr), skelHandler(nullptr), interactHandler(nullptr), audioHandler(nullptr){
			bool kinConn = (S_OK == initKinect());							
			if(!kinConn){	clog<<"Kinect Handler Log : Kinect not initialised (cstrctr) : "<<std::endl;	}	
			InitKinectStreams();	
		}//cnstrctr

		KinectHandler::~KinectHandler(void){	shutDown(true);}//dstrctr

		//shut down existing kinectSnsr and destroy all handlers/streams, to reinit clean.  if called from destructor, don't bother to re-init handler variables
		void KinectHandler::shutDown(bool dstrctr){
			clog<<"Kinect Handler Log : begin shutdown() :"<<std::endl;
	//		clog<<"Kinect Handler Log : attempt shutdown stream & delete imgHandler :"<<std::endl;
			if(nullptr != imgHandler) {delete imgHandler;	imgHandler = nullptr;}
	//		clog<<"Kinect Handler Log : attempt shutdown stream & delete dpthHandler :"<<std::endl;
			if(nullptr != dpthHandler) {delete dpthHandler; dpthHandler = nullptr;}
	//		clog<<"Kinect Handler Log : attempt shutdown stream & delete skelHandler :"<<std::endl;
			if(nullptr != skelHandler) {delete skelHandler; skelHandler = nullptr;}
	//		clog<<"Kinect Handler Log : attempt shutdown stream & delete audioHandler :"<<std::endl;
			if(nullptr != audioHandler) {delete audioHandler; audioHandler = nullptr;}
	//		clog<<"Kinect Handler Log : attempt shutdown stream & delete interactHandler :"<<std::endl;
			if(nullptr != interactHandler) {delete interactHandler; interactHandler = nullptr;}
	//		clog<<"Kinect Handler Log : attempt shutdown kinectSnsr :"<<std::endl;
			if(nullptr != kinectSnsr) {
				kinectSnsr->NuiShutdown();	
				kinectSnsr->Release();
				kinectSnsr = nullptr;
			}
			//re-init values of kinectHandler
			if(!dstrctr){
				flags.swap(vector<bool>(KH_NumFlags, false));			//sets all flags to be false
				reConnectCountDown = _RCD;
			}
			clog<<"Kinect Handler Log : shutdown() complete :"<<std::endl;
		}//shutdown

		//attempt to reconnect kinectSnsr unit
		bool KinectHandler::checkKinectConn(){
			//clog<<"Kinect Handler Log : Kinect connection verification begin"<<std::endl;
			bool badStatus = !this->kinValid();
			if(reConnectCountDown > 0 ){	--reConnectCountDown;} 
			else {
				reConnectCountDown = _RCD;
				if(badStatus){
					clog<<"Kinect Handler Log : Attempt shutdown "<<std::endl;
					shutDown(false);																		//shut down all streams and free all handlers if they've been previously initialized
					clog<<"Kinect Handler Log : Kinect not found, attempting reconnection "<<std::endl;
					badStatus = (S_OK != initKinect());												//attempt to reconnect
					if(!badStatus){
						clog<<"Kinect Handler Log : Kinect found, attempting reinitialization of streams "<<std::endl;
						HRESULT hr = InitKinectStreams();
						return (S_OK == hr);
					} else {						clog<<"Kinect Handler Log : No Kinect found"<<std::endl;	}		//if kinectSnsr found/reconnected, else
				}//if kinectSnsr is null
			}//reconnection timer
			return !badStatus;																		//return whether status is good
		}//checkKinectConn()

		//init streams and set flags for stream values
		//audio must be inited -after- skeleton stream
		HRESULT KinectHandler::InitKinectStreams(){
			if(this->kinValid()){
				clog<<"Kinect Handler Log : initialize camera tilt"<<std::endl;
				setTilt(0);
				clog<<"Kinect Handler Log : current camera tilt : "<<getTilt()<<std::endl;
				clog<<"Kinect Handler Log : attempt all streams init"<<std::endl;
				if(nullptr != imgHandler){ 			imgHandler->initKinDataStream();}
				else {								imgHandler = new KinImageHandler(kinectSnsr,this);}
				if(nullptr != dpthHandler){ 		dpthHandler->initKinDataStream();}
				else {								dpthHandler = new KinDepthHandler(kinectSnsr,this);}
				if(nullptr != skelHandler){ 		skelHandler->initKinDataStream();}
				else {								skelHandler = new KinSkelHandler(kinectSnsr,this);}
				if(nullptr != interactHandler){ 	interactHandler->initKinDataStream();}
				else {								interactHandler = new KinInteractHandler(kinectSnsr,this);}
				if(nullptr != audioHandler){ 		audioHandler->initKinDataStream();}
				else {								audioHandler = new KinAudioHandler(kinectSnsr, this);}
			
				//build avatar frame if just making skelhandler stream - local values will already be set - this is set as kinecthandler is first instantiated
				if((nullptr != skelHandler) && (!flags[_KH_SKH_AJNT_SET_IDX]) && (flags[_KH_SKH_AJNT_INIT_IDX] == true)){
					setFlag(_KH_SKH_AJNT_SET_IDX, true);
					skelHandler->buildFrameFromSkelAvatar();
				}
				setFlag(_KH_VLD_IMAGE_STRM,imgHandler->isValidStream);
				setFlag(_KH_VLD_DEPTH_STRM,dpthHandler->isValidStream);
				setFlag(_KH_VLD_SKEL_STRM, skelHandler->isValidStream);
				setFlag(_KH_VLD_NTRCT_STRM,interactHandler->isValidStream);
				setFlag(_KH_VLD_AUDIO_STRM,audioHandler->isValidStream);
				clog<<"Kinect Handler Log : all streams init finished"<<std::endl;
			} else {							clog<<"Kinect Handler Log : attempted all streams init failed - No Kinect present"<<std::endl;		return -1;}
			return S_OK;							//modify to reflect stream status
		}//InitKinectStreams

		//init kinectSnsr sensor
		HRESULT KinectHandler::initKinect(){
			// Get a working kinectSnsr sensor - just getting first sensor here.  ultimately could handle multiples
			int numSensors;
			INuiSensor* tmpKinect;
			HRESULT hr = NuiGetSensorCount(&numSensors);
			if (FAILED(hr)){return -1;}
			for(int i=0; i<numSensors; ++i){
				hr = NuiCreateSensorByIndex(i, &tmpKinect);
				if (FAILED(hr)){            continue;        }										//if didn't create sensor move on to next value
				hr = tmpKinect->NuiStatus();
				if(S_OK == hr){				kinectSnsr = tmpKinect; break;}							//good sensor, set global to this sensor, leave for loop
				tmpKinect->Release();																//release bad sensor		
			}//for numsensors
			if (nullptr != kinectSnsr){
				hr = kinectSnsr->NuiInitialize(NUI_INITIALIZE_FLAG_USES_AUDIO | 
											NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |		//PLAYER_INDEX needed for interactions functionality
											NUI_INITIALIZE_FLAG_USES_COLOR | 
											NUI_INITIALIZE_FLAG_USES_SKELETON );
				if(SUCCEEDED(hr)){//create events to signal data is available
					kinectSnsr->NuiSetForceInfraredEmitterOff(FALSE);								// Ensure infrared emitter enabled
				}
			}//if kinectSnsr != null
			if (nullptr == kinectSnsr || FAILED(hr)){	       clog<<"Kinect Handler Log :  No ready kinect found (initKinect())"<<std::endl;       return -1;   }
			//kinectSnsr->setSetDeviceStatusCallback();
			return hr;
		}//initKinect
	
		//////////////
		//handler passthrough methods - isolate handlers in case kinect gets unplugged and handlers get destroyed
		//		if ((nullptr == kinectSnsr) || (S_OK != kinectSnsr->NuiStatus())){	//if valid kinectSnsr - check if valid stream
		//////////////

			//format depth or RGB image for display in KinectController
		GLubyte* KinectHandler::buildImage(bool isDepth){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if(isDepth){	if(nullptr != dpthHandler){ 		return dpthHandler->KinStreamHandler::buildImage();}
					return NULL;
				} else {		if(nullptr != imgHandler){ 			return imgHandler->KinStreamHandler::buildImage();}
					return NULL;
				}
			}//if valid kinectSnsr - check if valid stream
			return NULL;
		}//buildImage

		GLuint KinectHandler::getImageTexID(bool isDepth){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if(isDepth){	if(nullptr != dpthHandler){ 		return dpthHandler->getImgTextureID();}
					return -1;
				} else {		if(nullptr != imgHandler){ 			return imgHandler->getImgTextureID();}
					return -1;
				}
			}//if valid kinectSnsr - check if valid stream
			return -1;
		}//getImageTexID
		
		/////////////////
		//Depth handler function calls
			//format depth or RGB image for display in KinectController
		GLubyte* KinectHandler::getDepthHandImg(bool isLeft){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if(nullptr != dpthHandler){ 	return dpthHandler->getDepthHandImg(isLeft);}
			}//if valid kinectSnsr - check if valid stream
			return NULL;
		}//getDepthHandImg

			//returns a value from 0 - 3, 1 left hand only, 2 right hand only, 3 both hands grip, 0 no grip
		int KinectHandler::getPressGripState(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != dpthHandler) {		return dpthHandler->getPressGripState();}}
			return 0;
		}//getPressGripState

		int KinectHandler::getHandImageWidth(bool left){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != dpthHandler){ 		return dpthHandler->getHandImageWidth(left);}}//if valid kinectSnsr - check if valid stream
			return -1;
		}//getHandImageWidth
		
		int KinectHandler::getHandImageHeight(bool left){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != dpthHandler){ 		return dpthHandler->getHandImageHeight(left);}}//if valid kinectSnsr - check if valid stream
			return -1;
		}//getHandImageHeight

			//raw skel locations mapped to depth space for both hands - idx's 0,1,2 for left hand, 3,4,5 for right (x,y depth), 6,7,8 for root, 9,10,11 for right shoulder
		Eigen::VectorXi KinectHandler::getDepthRawHandDepthPos(){										//returns raw hand locations converted from skel space to depth space
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != dpthHandler){			return dpthHandler->getDepthRawHandDepthPos();}}//if valid kinectSnsr, check if valid stream and stream has valid vals
			Eigen::VectorXi tv = Eigen::VectorXi(12);
			tv<<-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1;
			return tv;
		}//getDepthRawHandDepthPos	

		//////////////////
		//skelHandler function calls
		void KinectHandler::buildSkeleton(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != skelHandler){			skelHandler->buildSkeletonData();}}//if valid kinectSnsr - check if valid stream
		}//buildSkeleton

		vector<int>	KinectHandler::getCtlSkelJointState(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != skelHandler){			return skelHandler->getCtlSkelJointState();}}
			return vector<int>(NUI_NumJnts);
		}//getCtlSkelJointState

		vector<double> KinectHandler::getClippingBox(){
			vector<double> res(6);
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if(nullptr != skelHandler){			res = skelHandler->getClippingBox();}}//if valid kinectSnsr - check if valid stream
			return res;
		}//buildSkeleton

		void KinectHandler::buildSkelFrameFromSkelAvatar(const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& _IKHandles, vector<string>& _IKHandleNames){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){				//if valid kinectSnsr - check if valid stream
				_tmpSkelIKHandles = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(_IKHandles);						//setting local copies
				_tmpSkelIKHandleNames = vector<string>(_IKHandleNames);				//need to be set before skeleton handler is called, in case skelHandler is not instantiated yet
				setFlag(_KH_SKH_AJNT_INIT_IDX, true);
				if(nullptr != skelHandler){ 
					setFlag(_KH_SKH_AJNT_SET_IDX, true);
					skelHandler->buildFrameFromSkelAvatar();
				} 
			}
		}//buildFrameFromSkelAvatar

		bool KinectHandler::validCtlSkel(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != skelHandler){		return skelHandler->validCtlSkel();	}}
			return false;
		}//validCtlSkelAvail

		vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinectHandler::getLastKnownSkelJointLocs() {
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if((nullptr != skelHandler) && (flags[_KH_SKH_AJNT_SET_IDX])){ return skelHandler->getLastKnownSkelJointLocs();}			//if valid kinectSnsr, check if valid stream and stream has valid vals
				else if(!flags[_KH_SKH_AJNT_SET_IDX] && flags[_KH_SKH_AJNT_INIT_IDX]){	return _tmpSkelIKHandles;}}							//else, if not valid skel handler,
			return vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(NUI_NumJnts,vec3_zero);
		}//getLastKnownSkelJointLocs

		vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinectHandler::getKinSkelNoisyMarkers() {
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != skelHandler) { return skelHandler->getKinSkelNoisyMarkers();}}											//if valid kinectSnsr, check if valid stream and stream has valid vals
			return vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(NUI_NumJnts,vec3_zero);
		}//getKinSkelNoisyMarkers

		vector<string> KinectHandler::getJointAngleStrs(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != skelHandler ){ 			return skelHandler->getJointAngleStrs();	}	}//if valid kinectSnsr - check if valid stream
			return vector<string>(NUI_NumJnts, "");
		}//getJointAngleStrs

		vector<string> KinectHandler::getAvtrBoneLenVecStrs(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != skelHandler ){ 			return skelHandler->getAvtrBoneLenVecStrs();	}}//if valid kinectSnsr - check if valid stream
			return vector<string>(NUI_NumBones, "");
		}//getJointAngleStrs  processSpeechEvents

		vector<string> KinectHandler::getBestKinSkelJointLocsStrs(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != skelHandler ){ 			return skelHandler->getBestKinSkelJointLocsStrs();	}	}//if valid kinectSnsr - check if valid stream
			return vector<string>(NUI_NumJnts, "");
		}//getBestKinSkelJointLocs
	
		Eigen::Vector3d KinectHandler::getLeftHandPos() {	
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if((nullptr != skelHandler) && (flags[_KH_SKH_AJNT_SET_IDX])){ return skelHandler->getLastKnownSkelJointLocs()[jIdx_HandL];}}//if valid kinectSnsr, check if valid stream and stream has valid vals
			return Eigen::Vector3d(0,0,0);
		}//getLeftHandPos

		Eigen::Vector3d  KinectHandler::getRightHandPos() {	
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if((nullptr != skelHandler) && (flags[_KH_SKH_AJNT_SET_IDX])){ return skelHandler->getLastKnownSkelJointLocs()[jIdx_HandR];}}//if valid kinectSnsr, check if valid stream and stream has valid vals
			return Eigen::Vector3d(0,0,0);
		}//getRightHandPos

		deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinectHandler::getLeftHandMoveVel() {	
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if((nullptr != skelHandler) && (flags[_KH_SKH_AJNT_SET_IDX])){ return skelHandler->getJointVelsForJoint(jIdx_HandL);}}//if valid kinectSnsr, check if valid stream and stream has valid vals
			Eigen::Vector3d tmpVec = Eigen::Vector3d(0,0,0);
			return deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(jntPosAraDpth,tmpVec);
		}//getLeftHandMoveVec

		deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinectHandler::getRightHandMoveVel() {	
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if((nullptr != skelHandler) && (flags[_KH_SKH_AJNT_SET_IDX])){ return skelHandler->getJointVelsForJoint(jIdx_HandR);}}//if valid kinectSnsr, check if valid stream and stream has valid vals
			Eigen::Vector3d tmpVec = Eigen::Vector3d(0,0,0);
			return deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(jntPosAraDpth,tmpVec);
		}//getRightHandMoveVec

            //raw skel locations for both hands - idx's 0,1,2 for left hand, 3,4,5 for right (x,y z), root, 6,7,8, r shoulder 9,10,11
		Eigen::VectorXi KinectHandler::getSkelRawHandDepthPos(){										//returns raw hand locations converted from skel space to depth space
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if((nullptr != skelHandler) && (skelHandler->hasValidCtlSkel())){ return skelHandler->getSkelRawHandDepthPos();}}//if valid kinectSnsr, check if valid stream and stream has valid vals
			Eigen::VectorXi tv = Eigen::VectorXi(12);
			tv<<-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1;
			return tv;
		}//getSkelRawHandDepthPos

            //raw skel locations mapped to depth space for both hands - idx's 0,1,2 for left hand, 3,4,5 for right (x,y depth)
		Eigen::VectorXd KinectHandler::getSkelRawHandPos(){										//returns raw hand locations converted from skel space to depth space
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if((nullptr != skelHandler) && (skelHandler->hasValidCtlSkel())){ return skelHandler->getSkelRawHandPos();}}//if valid kinectSnsr, check if valid stream and stream has valid vals
			Eigen::VectorXd tv = Eigen::VectorXd(12);
			tv<<-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100;
			return tv;
		}//getSkelRawHandDepthPos
		////////////////////
		//interaction handler function calls
		void KinectHandler::buildInteractData(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != interactHandler){			interactHandler->buildInteractData();}}//if valid kinectSnsr - check if valid stream
		}//buildInteractData

			//called from depth stream to give interaction stream handler depth data frame.  Handler will hold data until interact handler needs it
		bool KinectHandler::ProcessDepth(unsigned int &LRsize, unsigned char* &pbits , LARGE_INTEGER &timestamp){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if(nullptr != interactHandler){			return interactHandler->ProcessDepth(LRsize, pbits, timestamp);		}
				else {							}//set local reference, to pass to interact handler when it is ready
			}
			return false;
		}//ProcessDepth

			//called from skel stream to give interaction stream handler depth skel frame.  Handler will hold data until interact handler needs it
		bool KinectHandler::ProcessSkeleton(NUI_SKELETON_FRAME& skelFrame){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	
				if(nullptr != interactHandler){			return interactHandler->ProcessSkeleton(skelFrame);		}
				else {							}//set local reference, to pass to interact handler when it is ready
			}
			return false;
		}//ProcessSkeleton

		////////////////
		//audio handler function calls
		void KinectHandler::processSpeechEvents(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != audioHandler){ 	audioHandler->processSpeech();	}}//if valid kinectSnsr - check if valid stream
		}//processSpeechEvents
			//clear out speech recognition values once speech has been processed by kinect controller
		void KinectHandler::clearSpeechVals(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != audioHandler){ 	audioHandler->clearSpeechVals();	}}//if valid kinectSnsr - check if valid stream
		}//clearSpeechVals
		double KinectHandler::getBeamAngle(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != audioHandler){			return audioHandler->getBeamAngle();		}	}//if valid kinectSnsr - check if valid stream
			return KH_AUD_ANGLE_FAIL;
		}//getBeamAngle

		double KinectHandler::getSourceAngle(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != audioHandler){			return audioHandler->getSourceAngle();		}	}//if valid kinectSnsr - check if valid stream
			return -KH_AUD_ANGLE_FAIL;
		}//getSourceAngle

		double KinectHandler::getSourceAngleConf(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != audioHandler){			return audioHandler->getSourceAngleConf();		}	}//if valid kinectSnsr - check if valid stream
			return -1;
		}//getSourceConf

		float KinectHandler::getSpchRecogConf(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != audioHandler){			return audioHandler->getSpchRecogConf();	}	}//if valid kinectSnsr - check if valid stream
			return -1;
		}//getSpchRecogConf

		string KinectHandler::getSpchRecogResult(){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != audioHandler){			return audioHandler->getSpchRecogResult();		}	}//if valid kinectSnsr - check if valid stream
			return "";
		}//getSpchRecogResult

		///////////////////
		//general stream handler function calls
		void KinectHandler::SetFlagsForStream(KinStreamHandler* strm, int idx, bool val){
			if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){	if(nullptr != strm){			strm->setFlag(idx,val);			}	}
		}//SetFlagsForStream


        /////////////////
        /// XML stream state flag setter functions

        bool KinectHandler::setKinAudioHandlerFlag(int idx, bool val){
            if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){if(nullptr != audioHandler){   audioHandler->setFlag(idx,val);  }}
            return true;
        }//	setKinAudioHandlerFlag

        bool KinectHandler::setKinDepthHandlerFlag(int idx, bool val){
            if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){if(nullptr != dpthHandler){    dpthHandler->setFlag(idx,val); }}
            return true;
        }//	setKinDepthHandlerFlag

        bool KinectHandler::setKinImageHandlerFlag(int idx, bool val){
            if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){if(nullptr != imgHandler){    imgHandler->setFlag(idx,val); }}
            return true;
        }//	setKinImageHandlerFlag

        bool KinectHandler::setKinInteractHandlerFlag(int idx, bool val){
            if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){if(nullptr != interactHandler){   interactHandler->setFlag(idx,val);   }}
            return true;
        }//setKinInteractHandlerFlag

        bool KinectHandler::setKinSkelHandlerFlag(int idx, bool val){
            if((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus())){if(nullptr != skelHandler){  skelHandler->setFlag(idx,val);  }}
            return true;
        }//setKinSkelHandlerFlag

		//////////////
		//end handler pass through methods
		//////////////

		//clears out the tracking flags for skeleton data
		void KinectHandler::clearSkelFlags(){
			flags[_KH_SKEL_TRACKED_IDX] = false;
			flags[_KH_SKEL_ROOTONLY_IDX] = false;
			flags[_KH_SKEL_NONEFOUND_IDX] = false;
			flags[_KH_SKH_VLD_JNTL_IDX] = false;
		}

		//camera tilt methods - don't use repeatedly, and allow to finish before calling again
		//will set the tilt based on the value passed 
		void KinectHandler::setTilt(int newTilt){
			camAngle = vldTiltAngle(long(newTilt));
			HRESULT hr = kinectSnsr->NuiCameraElevationSetAngle(camAngle);
			if(FAILED(hr)){	clog<<"Kinect Handler Log : Failed to initialize camera tilt angle"<<std::endl;}
			//setting tilt moves slowly, do not call often
		}//setTilt

		//will tilt the camera by amount passed
		void KinectHandler::modTilt(int deltaTilt){
			camAngle = vldTiltAngle(camAngle + deltaTilt);
			kinectSnsr->NuiCameraElevationSetAngle(camAngle);
			//setting tilt moves slowly, do not call often
		}//modTilt

		//get current tilt
		int KinectHandler::getTilt(){
			long tiltAngle;					//value goes in here
			kinectSnsr->NuiCameraElevationGetAngle(&tiltAngle);
			camAngle = tiltAngle;
			return int(tiltAngle);
		}//gettilt

		//will restrict any angle for the kinectSnsr tilt to be between hard bounds set in KinHndlrConsts
		long KinectHandler::vldTiltAngle(long reqAngle){ return (reqAngle < kinMaxTilt ? (reqAngle > kinMinTilt ? reqAngle : kinMinTilt) : kinMaxTilt);}

		//set flags for kinectSnsr handler's state machine
		void KinectHandler::setFlag(int idx, bool val){
			flags[idx] = val;
			switch (idx){
				case _KH_SEATED_IDX				: {
						SetFlagsForStream(imgHandler,		_IM_SEATED_IDX, val);
						SetFlagsForStream(dpthHandler,		_DP_SEATED_IDX, val);	
						SetFlagsForStream(skelHandler,		_SK_SEATED_IDX, val);	
						SetFlagsForStream(interactHandler,	_IH_SEATED_IDX, val);
						SetFlagsForStream(audioHandler,		_AU_SEATED_IDX, val);					
					break;}
				case _KH_NEAR_IDX				: {
						SetFlagsForStream(imgHandler,		_IM_NEAR_IDX, val);
						SetFlagsForStream(dpthHandler,		_DP_NEAR_IDX, val);	
						SetFlagsForStream(skelHandler,		_SK_NEAR_IDX, val);	
						SetFlagsForStream(interactHandler,	_IH_NEAR_IDX, val);
						SetFlagsForStream(audioHandler,		_AU_NEAR_IDX, val);					
					break;}
				case _KH_SKEL_TRACKED_IDX		: {
					if(val){				flags[_KH_SKEL_NONEFOUND_IDX] = false;		flags[_KH_SKEL_ROOTONLY_IDX] = false;}
					break;}
				case _KH_SKEL_ROOTONLY_IDX	: {
					if(val){				flags[_KH_SKEL_NONEFOUND_IDX] = false;		flags[_KH_SKEL_TRACKED_IDX] = false;}
					break;}
				case _KH_SKEL_NONEFOUND_IDX		: {
					if(val){				flags[_KH_SKEL_TRACKED_IDX] = false;	flags[_KH_SKEL_ROOTONLY_IDX] = false;}
					break;}
				case _KH_VLD_IMAGE_STRM		: {break;}		
				case _KH_VLD_DEPTH_STRM		: {break;}
				case _KH_VLD_SKEL_STRM		: {break;}
				case _KH_VLD_NTRCT_STRM		: {break;}
				case _KH_VLD_AUDIO_STRM		: {break;}
				case _KH_DPTH_DISP_HND		: {											//display hand info from depth handler
					SetFlagsForStream(dpthHandler,		_DP_DISP_HNDS, val);			//display hands in depth image
					SetFlagsForStream(dpthHandler,		_DP_BUILD_HNDIMG, val);			//build hand images from processed depth data
					break;}
				case _KH_SKEL_CALC_HND		: {				//calculate hand info
					SetFlagsForStream(dpthHandler, _DP_PROC_HNDS, val);
					break;}
				case _KH_DISP_DEBUG			: {break;}


			}//switch
		}//setFlag

		bool KinectHandler::kinConnected(){ return nullptr != kinectSnsr;}											//kinectSnsr is present
		bool KinectHandler::kinValid(){ return ((nullptr != kinectSnsr) && (S_OK == kinectSnsr->NuiStatus()));}		//kinectSnsr is present and valid
		bool KinectHandler::kinValidSkelAvail(){ 	return ((nullptr != kinectSnsr) && (flags[_KH_VLD_SKEL_STRM]) && ((flags[_KH_SKEL_TRACKED_IDX]) || (flags[_KH_SKEL_ROOTONLY_IDX])));		}
		bool KinectHandler::kinValidIKSkelAvail(){ 	return ((nullptr != kinectSnsr) && (flags[_KH_VLD_SKEL_STRM]) && (flags[_KH_SKEL_TRACKED_IDX]));		}
		bool KinectHandler::kinSkelHandlerAvail(){return (nullptr != kinectSnsr) && (flags[_KH_VLD_SKEL_STRM]);}	
		bool KinectHandler::kinSkelAvatarHndlsSet(){return flags[_KH_SKH_AJNT_SET_IDX];}	
		bool KinectHandler::kinAudioHandlerAvail(){return (nullptr != kinectSnsr) && (flags[_KH_VLD_AUDIO_STRM]);}

	}//namespace kinect
}//namespace rtql8
