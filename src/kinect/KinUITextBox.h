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

#ifndef __KINUITEXTBOX_H__
#define __KINUITEXTBOX_H__
#include "KinUIComponent.h"
/*
  describes a UI component for the kinect to interact with : 
  this is a simple 2d button that should respond to mouse click, as well as hand click or grab events
*/
using namespace std;

namespace rtql8 {
    namespace kinect{ 
        class KinUITextBox : public KinUIComponent{
        public:
            KinUITextBox():KinUIComponent(), txtVal(""), newTxtVal(""),tbState(0), maxChars(10), beingEdited(false), cursorOn(false), blinkCount(0){init();}
            KinUITextBox(int _x, int _y, int _w=0, int _h=0, float _sX = 0, float _sY=0, string _lbl="",string _msOverTxt="", string _stsTxt=""):
                KinUIComponent(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt,  _stsTxt), txtVal(""),newTxtVal(""), tbState(0),maxChars(10), beingEdited(false), cursorOn(false), blinkCount(0)	
                {	init();		}				//local init

            KinUITextBox(const KinUITextBox& _mB):KinUIComponent(_mB), txtVal(_mB.txtVal),newTxtVal(_mB.newTxtVal), tbState(_mB.tbState),
                maxChars(_mB.maxChars), beingEdited(_mB.beingEdited), cursorOn(_mB.cursorOn), blinkCount(_mB.blinkCount){}//should be copyswap
            ~KinUITextBox(void){}

            virtual int reInit();                                                                   //specifically designed to be called manually
            virtual void draw();
            virtual bool isClicked() {  
                bool retVal = flags[UIobjIDX_Click];    
                flags[UIobjIDX_Click] = false;  
                return retVal;   
            }        //returns whether this object has been clicked, and then clears click value

            bool isBeingEdited(){return beingEdited;}

            virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt){
                if(!this->isInsideHotSpot(_x, _y)){
                    beingEdited = false;
                    cursorOn = false;
                    blinkCount = 0;
                    txtVal = ("" == newTxtVal ? txtVal : string(newTxtVal));
                    newTxtVal = "";    
                }
            }//clear values if clearClicked-generating event happened within this object

            bool acceptInput(unsigned char ch);

            virtual int click(float _x, float _y, int cmpObj);
            virtual int drag(float _x, float _y){return -1;}

            virtual void getValueForEvent(int hand, int type, int drag, float& val);			
            virtual float getCurValue(){return -1;}	                                               //use to pass state? NOT USED FOR TB		
            string getCurTxtVal(){return (beingEdited ? newTxtVal : txtVal);}
            int getMaxchars(){return maxChars;}

            void setMaxChars(int _mx){maxChars = _mx;}

            virtual void buildLabel(){ }                                                           //modify labels on the fly
            
            KinUITextBox& operator=(KinUITextBox other){
                std::swap(static_cast<KinUITextBox>(*this), static_cast<KinUITextBox>(other));
                return *this;
            }

            friend void swap(KinUITextBox& _a, KinUITextBox& _b){
                using std::swap;
                swap(static_cast<KinUIComponent>(_a), static_cast<KinUIComponent>(_b));
                swap(_a.txtVal, _b.txtVal);
                swap(_a.newTxtVal, _b.newTxtVal);
                swap(_a.tbState, _b.tbState);
                swap(_a.maxChars, _b.maxChars);
                swap(_a.beingEdited, _b.beingEdited);
                swap(_a.cursorOn, _b.cursorOn);
                swap(_a.blinkCount, _b.blinkCount);
            }//swap

            friend std::ostream& operator<<(std::ostream& out, const KinUITextBox& cmp);

        private : //functions
            virtual void init();

        private : //variables
            std::string txtVal, newTxtVal;                                                      //current accepted text input, and current "being added" text input
            int tbState;                                                                        //state of text box - being edited? has been changed (i.e. dirty)?
            int maxChars;                                                                       //max number of input characters == max length of txtVal
			bool beingEdited;                                                                   //this text box is currently accepting input
            bool cursorOn;                                                                      //whether edit cursor is on or off
            int blinkCount;                                                                     //blink counter for cursor - set number of cycles on/off
        };//KinUITextBox
    }//namespace kinect
}//namespace rtql8
#endif
