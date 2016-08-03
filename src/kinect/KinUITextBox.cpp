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
#include "KinUITextBox.h"

using namespace std;

namespace rtql8{
    namespace kinect{

		
        int KinUITextBox::reInit(){			//any external-specific functionality
            KinUIComponent::reInit();		//parent reinit/init
            init();
            return 0;
        }

        void KinUITextBox::init(){
            flags[UIobjIDX_CanType] = true;
            flags[UIobjIDX_OffsLbl] = true;//always offset label for textboxes
            txtVal = "";
            newTxtVal = "";
            tbState = 0;
            beingEdited = false;
            cursorOn = false;
            maxChars = 10;  //default accept 10 chars
        }

        //current textbox text displayed, put cursor at end of text, accept new text. 
        //esc from ch ignores input, enter processes it by adding it to existing txtVal string
        //newTxtVal will enter this for the first time having a copy of the existing text in the textbox
        bool KinUITextBox::acceptInput(unsigned char ch){
            switch(ch){
                case(27):{//esc key - discard all new text entered - retain old txtVal
                    newTxtVal = "";
                    beingEdited = false;                //end editing
                    cursorOn = false;
                    break;}
                case(13):{//enter pressed - save modified text as the txtVal
                    txtVal = string(newTxtVal);
                    newTxtVal = "";
                    beingEdited = false;                //end editing
                    cursorOn = false;
                    break;}
                case(8):            //backspace
                case(127):{         //delete
                    if(0 < newTxtVal.length()){//remove char from end of newTxtVal if there are chars to remove
                        newTxtVal = string(newTxtVal,0,newTxtVal.length()-1);
                    } 
                    break;}
                default : {
                    if(maxChars > newTxtVal.length()){
                        stringstream ss;
                        ss<<newTxtVal;
                        ss<<ch;
                        newTxtVal = ss.str();
                    }
                }
            }//switch
            return beingEdited;
        }//acceptInput

        //draw this component on the screen
        void KinUITextBox::draw(){
            float lblX, lblY;
            if(flags[UIobjIDX_DispHS]){drawHotSpot();}
            //draw label offset
            int labelLen = label.length();
            lblX = (x < w ?  w :  labelLen * 14 * -1  );   //check if close to left edge
            lblY = h/8.0f;
            KinUIComponent::drawBaseCmp(lblX, lblY);
            //draw text inside box
            if(beingEdited){
                stringstream ss;
                ss<<newTxtVal;
                blinkCount++;
                int maxBCnt = 15;
                if((maxBCnt-1) == (blinkCount % maxBCnt)){            blinkCount = 0;         cursorOn = !cursorOn;       }
                if(cursorOn){       ss<<"|";      }         //blink pipe to act as cursor
                this->drawText(0,0,ss.str(), true, .2f);
            } else {
                this->drawText(0,0, txtVal, true, .2f);
            }
        }//draw

        //handles kinect event
        void KinUITextBox::getValueForEvent(int hand, int type, int drag, float& val){        }	

        //handles kinect event
        int KinUITextBox::click(float _x, float _y, int cmpObj){	
            if(this->isInsideHotSpot(_x, _y)){
                beingEdited = true;
                cursorOn = true;
                blinkCount = 0;
                newTxtVal = string(txtVal);             //make copy of current text in textbox
            } 
            else {
                beingEdited = false;
                cursorOn = false;
                blinkCount = 0;
                newTxtVal = "";                       
            }
            return ID;
        }

        std::ostream& operator<<(std::ostream& out, const KinUITextBox& cmp){
            out<<static_cast<const KinUIComponent&>(cmp);                                                         //start with base class
            out<<"\tText box : Current text : "<<cmp.txtVal<<endl;
            out<<"\t Max Length : "<<cmp.maxChars<<" | Current state : "<<cmp.tbState<<" | Currently being edited : "<<cmp.beingEdited<<endl;
            out<<"\t Current edit text : '"<<cmp.newTxtVal<<"'"<<endl;
            return out;
        }//op<<

    }//namespace kinect
}//namespace rtql8
