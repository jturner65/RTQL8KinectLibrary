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

#ifndef __KINUIIMAGEBUTTON_H__
#define __KINUIIMAGEBUTTON_H__
#include "KinUIComponent.h"
/*
  describes a UI component for the kinect to interact with
  this is a button designed with the capability of displaying an image or sequence of images (animation).
  it is primarily intended to be used in 3d space, where it can display information in the background and allow
  the user to choose it via hand movement in avatar/skeleton space.
*/
using namespace std;

namespace rtql8 {
    namespace kinect{ 
        class KinUIImageButton : public KinUIComponent{
        public:
            KinUIImageButton():KinUIComponent(), imgData(0), imgTextureID(0), imgClickTextureID(0), useAnimLoop(false), useFileSystem(false), imgLoaded(false),animLoop(),loopTexIDs(), drawIdx(0), numFrames(0) {init();}
            KinUIImageButton(int _x, int _y, int _w=0, int _h=0, float _sX = 0, float _sY=0, string _lbl="",string _msOverTxt="", string _stsTxt=""):                                                  
                KinUIComponent(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt,  _stsTxt), imgData(0), imgTextureID(0),imgClickTextureID(0), useAnimLoop(false), useFileSystem(false), imgLoaded(false), animLoop(),loopTexIDs(), drawIdx(0), numFrames(0){	init();		}				    //local init
            KinUIImageButton(const KinUIImageButton& _mB):KinUIComponent(_mB), 
                imgData(_mB.imgData), imgTextureID(_mB.imgTextureID),imgClickTextureID(_mB.imgClickTextureID), useAnimLoop(_mB.useAnimLoop), useFileSystem(_mB.useFileSystem), imgLoaded(_mB.imgLoaded), animLoop(_mB.animLoop),loopTexIDs(_mB.loopTexIDs), drawIdx(_mB.drawIdx), numFrames(_mB.numFrames){}                                         //should be copyswap
            ~KinUIImageButton(void){}

            virtual int reInit();																				//specifically designed to be called manually
            virtual void draw();
            void drawImageButton();
            void drawAnimButton();

            void instImgBtn();                                                                                  //instantiate image button, when we are certain there is an opengl context extant

            virtual void clearClicked(float _x, float _y, int _srcOfEvnt){}
            virtual int click(float _x, float _y, int cmpObj);
            virtual int drag(float _x, float _y){return -1;}
            virtual bool isClicked() { 
                bool retVal = flags[UIobjIDX_Click];
                flags[UIobjIDX_Click] = false;
                return retVal;
            }
            virtual void getValueForEvent(int hand, int type, int drag, float& val);									
            virtual float getCurValue(){return 1;}							    //button will just provide single value with default event

            void setImgSrcFileName(string _src);                      

            void setImgTextureID(GLuint _imgTextureID){imgTextureID = _imgTextureID;}
            void setImgData(GLubyte* _imgData){imgData = _imgData;}
            void addAnimLoopImg(GLuint _imgTextureID,GLubyte* _imgData){
                numFrames++;
                animLoop.push_back(_imgData);
                loopTexIDs.push_back(_imgTextureID);
            }

            void setUseAnimLoop(bool _uLp){useAnimLoop = _uLp;}
            KinUIImageButton& operator=(KinUIImageButton other){
                std::swap(static_cast<KinUIImageButton>(*this), static_cast<KinUIImageButton>(other));
                return *this;
            }

            virtual void buildLabel(){ }                                                                                    //modify labels on the fly

            friend void swap(KinUIImageButton& _a, KinUIImageButton& _b){
                using std::swap;
                swap(static_cast<KinUIComponent>(_a), static_cast<KinUIComponent>(_b));
                swap(_a.imgData, _b.imgData);
                swap(_a.imgTextureID, _b.imgTextureID);
                swap(_a.imgClickTextureID, _b.imgClickTextureID);
                swap(_a.useAnimLoop, _b.useAnimLoop);
                swap(_a.useFileSystem, _b.useFileSystem);
                swap(_a.imgLoaded, _b.imgLoaded);
                swap(_a.animLoop, _b.animLoop);
                swap(_a.loopTexIDs, _b.loopTexIDs);
                swap(_a.drawIdx, _b.drawIdx);
                swap(_a.numFrames, _b.numFrames);
            }//swap

            friend std::ostream& operator<<(std::ostream& out, const KinUIImageButton& cmp);

        private : //functions
            virtual void init();
            bool loadImg(GLuint texID);

            void drawImgBtnFrmFS();
            void drawImgBtnFrmKin();

        private : //variables
            GLubyte* imgData;                       //an array of glubytes representing the image to display on the button
            GLuint imgTextureID;                    //the texture id corresponding to the image this button displays
            GLuint imgClickTextureID;               //texture to display when pressing - grayscale version of regular picture
            string imgSrc;                          //directory source for image
            bool useAnimLoop;                       //whether or not this button should animate through a loop of images TODO
            bool useFileSystem;                     //load image from file system - uses SOIL library
            bool imgLoaded;                         //if an appropriate image is loaded
            vector<GLubyte*> animLoop;              //an animation loop to display 
            vector<GLuint> loopTexIDs;              //the texture ID's of each image in the loop
            int drawIdx, numFrames;                 //the currently drawing frame of an animation loop, and the total frames in the loop
        };//KinUIImageButton
    }//namespace kinect
}//namespace rtql8
#endif
