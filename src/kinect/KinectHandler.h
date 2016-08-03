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

#ifndef __KINECTHANDLER_H__
#define __KINECTHANDLER_H__
/**
	author : john turner
	class to handle kinect interaction with IK/char controller program
	requires Microsoft kinect SDK 1.7, kinect interactions, and the speech sdk that accompanies the kinect sdk
**/
#include "KinHndlrConsts.h"

using namespace std;

namespace rtql8 {
	//need to use kinect to determine user's left and right hand locations, to manipulate skeleton
	namespace kinect{

	//forward declrs for classes
	class KinStreamHandler;									//base class for all stream handlers
	class KinImageHandler;
	class KinDepthHandler;
	class KinSkelHandler;
	class KinInteractHandler;
	class KinAudioHandler;

	class KinectHandler{
	public:
		KinectHandler();
		~KinectHandler();

		void shutDown(bool dstrctr);

		bool checkKinectConn();
		HRESULT initKinect();
		HRESULT InitKinectStreams();					//attempt to reinitialize data streams if kinect is unplugged or replugged in, to avoid segfaults

		void clearSkelFlags();

		//passthrough methods - do not call stream handlers directly as they may not successfully instantiate - all calls to streams must go through this class
		GLubyte* buildImage(bool isDepth);
		GLuint getImageTexID(bool isDepth);

		//depth handler functions
		GLubyte* getDepthHandImg(bool isLeft);
		int getPressGripState();
		int getHandImageWidth(bool left);
		int getHandImageHeight(bool left);
		Eigen::VectorXi getDepthRawHandDepthPos();

		//skelHandler function calls
		void buildSkeleton();
		vector<int>	getCtlSkelJointState();
		vector<double> getClippingBox();
		void buildSkelFrameFromSkelAvatar(const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& _IKHandles, vector<string>& _IKHandleNames);
		bool validCtlSkel();
		vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getLastKnownSkelJointLocs();
		vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getKinSkelNoisyMarkers();
		vector<string> getJointAngleStrs();
		vector<string> getAvtrBoneLenVecStrs();
		vector<string> getBestKinSkelJointLocsStrs();
		Eigen::Vector3d getLeftHandPos();
		Eigen::Vector3d  getRightHandPos();
		deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getLeftHandMoveVel() ;
		deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getRightHandMoveVel();
		Eigen::VectorXi getSkelRawHandDepthPos();
		Eigen::VectorXd getSkelRawHandPos();

		//interaction handler function calls
		void buildInteractData();
		bool ProcessDepth(unsigned int &LRsize, unsigned char* &pbits , LARGE_INTEGER &timestamp);
		bool ProcessSkeleton(NUI_SKELETON_FRAME& skelFrame);

		//audio handler function calls
		void processSpeechEvents();
		void clearSpeechVals();
		double getBeamAngle();
		double getSourceAngle();
		double getSourceAngleConf();
		float getSpchRecogConf();
		string getSpchRecogResult();

		//general stream handler function calls
		void SetFlagsForStream(KinStreamHandler* strm, int idx, bool val);

        /// XML stream state flag setter functions

        bool setKinAudioHandlerFlag(int idx, bool val);	
        bool setKinDepthHandlerFlag(int idx, bool val);	
        bool setKinImageHandlerFlag(int idx, bool val);	
        bool setKinInteractHandlerFlag(int idx, bool val);
        bool setKinSkelHandlerFlag(int idx, bool val);

		int getTilt();
		bool getFlag(int idx){return flags[idx];}

		void setFlag(int, bool);
		void setTilt(int newTilt);

		void modTilt(int deltaTilt);
		long vldTiltAngle(long reqAngle);

		bool kinConnected();							//kinect is present
		bool kinValid();								//kinect is present and valid
		bool kinValidSkelAvail();						//tracked skeleton or tracked skeleton root is available
		bool kinValidIKSkelAvail();						//tracked full skeleton available to be IK'ed to
		bool kinSkelHandlerAvail();						//skelHandler has been instantiated
		bool kinSkelAvatarHndlsSet();					//skelHandler has current values of avatar handle locations

		bool kinAudioHandlerAvail();					//skelHandler has been instantiated

		//variables
	public : 
		INuiSensor* kinectSnsr;								//sensor
		vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> _tmpSkelIKHandles;
		vector<string> _tmpSkelIKHandleNames;
		
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private : 
		vector<bool> flags;								//state flags - see template spec
		int reConnectCountDown;
		long camAngle;

		kinect::KinImageHandler*		imgHandler;
		kinect::KinDepthHandler*		dpthHandler;
		kinect::KinSkelHandler*			skelHandler;
		kinect::KinInteractHandler*		interactHandler;
		kinect::KinAudioHandler*		audioHandler;

        };//class KinectHandler
	}//kinecthandler namespace
}//namespace rtql8

#endif