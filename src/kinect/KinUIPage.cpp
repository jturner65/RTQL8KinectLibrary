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
#include "KinUIPage.h"

using namespace std;

namespace rtql8{
    namespace kinect{
        void KinUIPage::addUIButton(KinUIButton* btn) {
            std::shared_ptr<KinUIButton>    tmp0 (btn);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUIBtn.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }

        void KinUIPage::addUISlider(KinUISlider* sld) {
            std::shared_ptr<KinUISlider>    tmp0 (sld);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUISldr.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }
        
        void KinUIPage::addUICrank(KinUICrank* crk){
            std::shared_ptr<KinUICrank>    tmp0 (crk);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUICrnk.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }

        void KinUIPage::addUIImgBtn(KinUIImageButton* ibtn){
            std::shared_ptr<KinUIImageButton>    tmp0 (ibtn);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUIImgBtn.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }
        
        void KinUIPage::addUIPrgBar(KinUIPrgBar* pbar){
            std::shared_ptr<KinUIPrgBar>    tmp0 (pbar);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUIProgBar.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }

        void KinUIPage::addUITimer(KinUITimer* tmr){
            std::shared_ptr<KinUITimer>    tmp0 (tmr);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUITimer.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }

        void KinUIPage::addUITextBox(KinUITextBox* tbx){
            std::shared_ptr<KinUITextBox>    tmp0 (tbx);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUITextBox.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }

        void KinUIPage::addUICaption(KinUICaption* cap){
            std::shared_ptr<KinUICaption>    tmp0 (cap);
            std::shared_ptr<KinUIComponent> tmp1 = tmp0;
            KinPgUICaption.push_back(tmp0);
            KinPgUICmp.push_back(tmp1);
        }

        //called from kinectController :: TODO eventually - all page-specific event handling should be handled here, would address problems and inconsistencies with hand handling
        bool KinUIPage::checkUIElements(int& x, int& y, int& d, bool mouseEvent, int hand, int evnt){
            bool objFound = false;
            //if(mouseEvent) { 
            //    //mouse events : mouse action has triggered event
            //    bool msDragEvent = (2 == evnt) ? true : false;															//drag
            //    bool msClickEvent = (1 == evnt) ? true : false;												            //handle grip differently - 0:click up, 1:click down	
            //    for(int idx = 0; idx < KinPgUICmp.size(); ++idx){                                                                                                                        
            //        int clkVal = (KinPgUICmp[idx]->isInside(x,y));                                                        //value returned from isInside - what part of object click was inside
            //        if((0 != clkVal) || (KinPgUICmp[idx]->checkDragByThisCntrl(KC_MOUSE))){                               //check if inside this object, or if this object is currently being dragged
            //            if(!msDragEvent) {
            //                if(msClickEvent){   KinPgUICmp[idx]->setClicked(x,y,clkVal, KC_MOUSE);}
            //                else {              KinPgUICmp[idx]->clearClicked(x,y, KC_MOUSE);}                            //upon click up clear click event - needs to happen cycle after "click release" has been processed
            //            } else {
            //                int resD = KinPgUICmp[idx]->setDragged(msDragEvent,x,y, KC_MOUSE);
            //                if (resD != -1){      KinPgUICmp[idx]->stickToClick(x,y); }
            //            }										                                                        //set drag state to be whether or not this is a drag event						
            //            objFound = true;	
            //        } else { KinPgUICmp[idx]->clearClicked(x,y, KC_MOUSE);}									            //use location = -1,-1 for click release of object with mouse outside of object (location ignored)
            //    }//for each component
            //} else {																								    //hand ui event
            //    //hand events
            //    //bool hndDragEvent = (2 == evnt)? true : false;                                                        //tmp change to hand handling - push acts as drag too               
            //    bool hndDragEvent = (1 == evnt) || (2 == evnt)? true : false;                        
            //    //bool hndDragEvent = this->KC_flags[(KC_LEFT == hand ? _KC_LHAND_DRAG_OBJ : _KC_RHAND_DRAG_OBJ)];        //whether this hand is currently dragging
            //    bool hndClickEvent = (1 == evnt) || (2 == evnt) ? true : false;										    //need to treat a grab like a click and a grab
            //    for(int idx = 0; idx < KinPgUICmp.size(); ++idx){	
            //        int clkHSVal = (KinPgUICmp[idx]->isInsideHotSpot(x,y));                                                     //just check if inside hotspot location at all - will return 1 if button, 2 if crank or slider (denoting bar or thumb)
            //        if((0 != clkHSVal) || (KinPgUICmp[idx]->checkDragByThisCntrl(hand))){                                       //verify that this hand is the hand generating the event
            //            clog<<"hand : "<< (KC_LEFT == hand ? " Left" : " Right")<< (hndDragEvent ? " dragged " : " clicked ")<<"in object IDX: "<<idx<<" event : "<<evnt<<" clickEvent : "<<hndClickEvent<<" hotspot val : "<<clkHSVal<<" x,y : ("<<x<<","<<y<<")"<<std::endl;
            //            if(hndClickEvent){		
            //                KinPgUICmp[idx]->setClicked(x,y,clkHSVal, hand);
            //                if(0 != clkHSVal){
            //                    //this->setFlag((KC_LEFT == hand ? _KC_LHAND_DRAG_OBJ : _KC_RHAND_DRAG_OBJ), true);
            //                    this->KinHands[hand].setFlags(_KHnd_HAND_DRAG_OBJ,true);
            //                    this->KinHands[hand].setFlags(_KHnd_HAND_CLCKD,true);
            //                }    //if being dragged by this input, set to true
            //                int resD = KinPgUICmp[idx]->setDragged(hndDragEvent,x,y, hand);       
            //                if (resD != -1){
            //                    clog<<(KC_LEFT == hand ? "Left" : "Right")<<" hand dragging event : "<<hndDragEvent<<std::endl;
            //                    int tmpX = x, tmpY = y;
            //                    KinPgUICmp[idx]->stickHandToClick(tmpX,tmpY);                       //tmpX,tmpY get set with position where hand should be moved to while dragging
            //                    this->KinHands[hand].setNewHandLoc2d(tmpX, tmpY);                   //set new location for this hand
            //                }                                  //force hand to stay inside object while click/grab engaged                               
            //            } else 	{	KinPgUICmp[idx]->clearClicked(x,y, hand);}
            //            objFound = true;	
            //        } else { 
            //            KinPgUICmp[idx]->clearClicked(x,y, hand);
            //        }												//use location = -1,-1 for click release of object with mouse outside of object
            //    }//for each object
            //    this->KinHands[hand].setFlags(_KHnd_HAND_IN_OBJ, objFound);
            //    if((0==evnt) || (!(objFound))){       
            //        //this->setFlag((KC_LEFT == hand ? _KC_LHAND_DRAG_OBJ : _KC_RHAND_DRAG_OBJ), false);  
            //        this->KinHands[hand].setFlags(_KHnd_HAND_DRAG_OBJ,false);
            //    }    //clear drag flag corresponding to this hand if we're not in an object                
            //}//if event from mouse else from kin hand
            return objFound;
        }//checkUIElements        
    }//namespace kinect
}//namespace rtql8

