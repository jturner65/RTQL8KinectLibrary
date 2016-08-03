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
#include "KinStreamHandler.h"


using namespace std;

namespace rtql8{
	namespace kinect{
		
		unsigned int KinStreamHandler::ID_gen = 0;					//used to generate unique id's for all stream handlers

		KinStreamHandler::KinStreamHandler(INuiSensor* _oKin, KinectHandler* _owningKH, handlerType _type): ID(ID_gen++),
											owningKinect(_oKin), owningKH(_owningKH), type(_type), isValidStream(false), KinStreamHandle(NULL)									
		{
			initHndlrDataStructs();
		}//cnstrctr	

		void KinStreamHandler::initHndlrDataStructs(){
			initVars();												//init base class vars here
		}//initHndlrDataStructs

		void KinStreamHandler::initVars(){
			//clog<<"Base Class Log : init vars for stream : "<<getTypeName()<<std::endl;
			std::stringstream ss(stringstream::in | stringstream::out);
			ss<<"EVT_SH_#";
			ss<<ID;
			ss<<"_T_";
			ss<<getTypeName();
			string tmpStr = ss.str();
			std::wstring stemp = std::wstring(tmpStr.begin(), tmpStr.end());
			LPCWSTR eventName = stemp.c_str();
			clog<<"Base Class Log : "<<getTypeName()<<" stream event init name : "<<ss.str()<<"|"<<eventName<<std::endl;
			//using kinect event model - needs to be manual reset true
			m_hNextEvent = CreateEventW(NULL, TRUE, FALSE, eventName);		//security, manual reset required, initial state, event name
			//m_hNextEvent = CreateEvent(NULL, TRUE, FALSE, NULL);		//security, manual reset required, initial state, event name
			imgTextureID = initTexture();  
		}//initVars

		//initialize texture handle for depth and image streams
		GLuint KinStreamHandler::initTexture(){
			GLuint _TextureIDloc;
			glGenTextures(1, &_TextureIDloc);  
			return _TextureIDloc;
		}//initDepthTexture

		HRESULT KinStreamHandler::initKinStreamHandler(){//overridden by individual stream handlers
			initVars();													//initialize base-class stream variables
			if(owningKinect != NULL){
				HRESULT _StrmInit = this->initKinDataStream();			//derived class initialisation
				isValidStream = (SUCCEEDED(_StrmInit));
				clog<<"Base Class Log : "<<getTypeName()<<" stream init : "<<(isValidStream ? "true" : "false")<<" : "<<_StrmInit<<std::endl;
				return _StrmInit;
			} 
			isValidStream = false;
			clog<<"Base Class Log : No Valid Kinect found - no "<<getTypeName()<<" stream initialized"<<std::endl;
			return -1;
		}//initKinStreamHandler

		string KinStreamHandler::getTypeName(){
			switch (type){
				case _H_RGB		: {		return "RGB Image";}
				case _H_DEPTH	: {		return "Depth Image";}
				case _H_SKEL	: {		return "Skeleton Data";}
				case _H_INTERACT: {		return "Interaction Data";}
				case _H_AUDIO	: {		return "Audio Data";}
				default : {return "NONE";}
			}//switch
			return "NONE";
		}//getTypeName

		string KinStreamHandler::getTypeAbbrev(){
			switch (type){
				case _H_RGB		: {		return "RGB";}
				case _H_DEPTH	: {		return "DPTH";}
				case _H_SKEL	: {		return "SKEL";}
				case _H_INTERACT: {		return "INTR";}
				case _H_AUDIO	: {		return "AUD";}
				default : {return "NONE";}
			}//switch
			return "NONE";
		}//getTypeName

		//sets whether we are seated or not, returns if successful TODO: needs to actually restart specific streams for new mode
		bool KinStreamHandler::setSeatedMode(bool _seated){
			if((NULL != owningKinect) && (isValidStream)){				//we have a kinect and we have a valid stream
				owningKH->setFlag(_KH_SEATED_IDX,_seated);
				HRESULT _StrmSetRes = this->setStreamEnableFlags();	
				return (SUCCEEDED(_StrmSetRes));
			}
			return false;
		}//changeSeatedMode

		//sends whether we are seated or not, returns if successful TODO: needs to actually restart specific streams for new mode
		bool KinStreamHandler::setNearMode(bool _nearVal){
			if((NULL != owningKinect) && (isValidStream)){				//we have a kinect and a valid stream
				owningKH->setFlag(_KH_NEAR_IDX,_nearVal);
				HRESULT _StrmSetRes = this->setStreamEnableFlags();	
				return (SUCCEEDED(_StrmSetRes));
			}
			return false;
		}//changeNearMode

		GLuint KinStreamHandler::getImgTextureID(){			return imgTextureID;}
			//called from kinectHandler
		GLubyte* KinStreamHandler::buildImage(){
			this->buildImage(imgData);
			return imgData;
		}
		//debug method to show binary data in u of certain # of bits bit
		void KinStreamHandler::show_binary(unsigned int u, int bit){ 
			for(int t = static_cast<unsigned int>(std::pow(2.0f,bit)); t > 0; t /= 2){		if(u & t) {clog << "1 "; }  else {clog << "0 ";}}  clog << "\n"; }

	}//namespace kinecthandler
}//namespace rtql8