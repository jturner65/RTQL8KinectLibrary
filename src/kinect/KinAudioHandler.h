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

#ifndef __KINAUDIOHANDLER_H__
#define __KINAUDIOHANDLER_H__
/**
	author : john turner
	class to handle kinect audio interaction
	requires Microsoft SDK 1.7
**/

#include "KinStreamHandler.h"
#include "KinAudioStream.h"

using namespace std;

namespace rtql8 {
	namespace kinect{
		class KinectHandler;
		class KinStreamHandler;
		class KinAudioHandler : public KinStreamHandler {
		public:
			KinAudioHandler(INuiSensor* _oKin, KinectHandler* _owningKH);
			~KinAudioHandler(void);

			HRESULT initKinStreamHandler();
			HRESULT initKinDataStream();
			unsigned long getKHSMFlagMode(int idx, bool state);
			void initHndlrDataStructs();

			void processSpeech();
			string MapSpeechTag(LPCWSTR pszSpeechTag, float _conf);

			HRESULT InitializeAudioStream();
			HRESULT CreateSpeechRecognizer();
			HRESULT LoadSpeechGrammar();
			HRESULT StartSpeechRecognition();
			void setMediaType(DMO_MEDIA_TYPE& mt, WAVEFORMATEX& wfxOut);

			//get speech recog result, listening beam angle, source of audio angle, and confidence of source of audio angle, confidence of speech recognition result
			//string getSpchRecogResult();
			string getSpchRecogResult();
			double getBeamAngle();
			double getSourceAngle();
			double getSourceAngleConf();
			float getSpchRecogConf();

			//set listening beam based on location of current tracked skeleton - called from skeleton handler
			void setCurrBeamAngle(double _bA){		currBeamAngle = _bA;}
			void setBeamAngle(double beamAngle);
			//set source angle and source angle confidence - called from audio stream
			void setSourceAngle(double _sa){		currSourceAngle = _sa;}
			void setSourceAngleConf(double _sac){	currSourceConf = _sac;}

			//clear out speech values
			void clearSpeechVals(){currResult = ""; currConf = 0;}

			virtual bool setFlag(int idx, bool val);

		private : //methods
			void initVars();

		public : //variables
			INuiAudioBeam*	kinNuiAudSrc;				// audio source from spkr array - can get beam angle from here
			double			currBeamAngle,				// where sensor is listening
							currSourceAngle,			// where sensor perceives sound location is
							currSourceConf;				// confidence of sensor sound location	


		private : //variables

			// Audio stream captured from Kinect.
			KinAudioStream*	kinAudioStream;				// Audio stream captured from Kinect.
			ISpStream*      kinSpchStream;				// Stream given to speech recognition engine
			ISpRecognizer*  spchRecognizer;				// Speech recognizer
			ISpRecoContext* spchRecogCntxt;				// Speech recognizer context
			ISpRecoGrammar* spchRecogGrammar;			// Speech grammar
			string			currResult;					// most recent recognized speech result
			float			currConf;					// confidence of most recent recognized speech result
		};//KinAudioHandler class
	}//namespace kinect
}//namespace rtql8 

#endif

