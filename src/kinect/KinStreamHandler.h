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

#ifndef __KINSTREAMHANDLER_H__
#define __KINSTREAMHANDLER_H__


#include "KinectHandler.h"
#include "KinHndlrConsts.h"						//remove from this file when strmImgW and strmImgH can be "found"

/**
   author : john turner
   parent class for kinect stream handlers
   requires Microsoft SDK 1.7
**/

using namespace std;

namespace rtql8 {
    namespace kinect{

        class KinectHandler;

        enum handlerType{NONE=-1, _H_RGB, _H_DEPTH, _H_SKEL, _H_INTERACT, _H_AUDIO};

        class KinStreamHandler{
        public:
            KinStreamHandler(INuiSensor* _oKin, KinectHandler* _owningKH, handlerType _type);
            virtual ~KinStreamHandler(void){}

            bool setSeatedMode(bool seated);
            bool setNearMode(bool nearVal);
            virtual bool setFlag(int idx, bool val){return this->setFlag(idx,val);}		//call child class version

            string getTypeName();
            string getTypeAbbrev();
            GLuint getImgTextureID();

            virtual HRESULT setStreamEnableFlags(){return 0l;}
            virtual HRESULT initKinStreamHandler();
            virtual HRESULT initKinDataStream(){return S_FALSE;}
            virtual void initHndlrDataStructs();											//overridden in derived classes

            void setFlagVals(vector<bool>& flags);

        protected : 
            virtual void initVars();
        public :
            virtual unsigned long getKHSMFlagMode(int idx, bool stream){return 0l;}			//flags are DWORD = unsigned long - different flags for each stream

            GLuint initTexture();
            virtual GLubyte* buildImage(GLubyte* imgData){return imgData;}
            virtual void show_binary(unsigned int u, int bit);//debug method
            GLubyte* buildImage();

            //variables
            int ID;											//give each stream unique id
            static unsigned int ID_gen;

            handlerType type;
            bool isValidStream;

            GLuint imgTextureID;														// ID of the texture to contain Kinect Depth or image Data
            GLubyte imgData[strmImgW * strmImgH*4];										// BGRA array containing the texture data
            INuiSensor* owningKinect;													//owning kinect
            KinectHandler* owningKH;													//owning handler
            HANDLE KinStreamHandle;														//kinect stream handle
            HANDLE m_hNextEvent;														//event handle for this stream

        protected : 
            vector<bool> STRM_Flags;													//internal state machine flags - only access KinectHandler flags, and set these from modifcations to those


        };
    }//namespace kinect
}//namespace RTQL8
#endif
