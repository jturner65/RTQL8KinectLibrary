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
#include "KinUISlider.h"

using namespace std;

namespace rtql8{
    namespace kinect{
		
        int KinUISlider::reInit(){	
            KinUIComponent::reInit();		//parent reinit/init
            init();
            return 0;
        }

        void KinUISlider::init(){
            //initial values for various properties
            flags[UIobjIDX_CanDrag] = true;
        }
        //draw this component on the screen
        void KinUISlider::draw(){
            float trnX, trnY, lblX, lblY;								        //location of slider thumb, based on value of slider,with 0 putting at top/left, and 1 placing at bottom/right, and label above thumb
            glPushMatrix();
                if(flags[UIobjIDX_DispHS]){drawHotSpot();}
                drawSliderBar();
            glPopMatrix();
            glPushMatrix();
                trnX = (horiz ? -(w/2.0) + slidePos * sbDimL : 0);              
                trnY = (horiz ? 0 : -(h/2.0) + (slidePos * sbDimL) );
                lblX = (horiz ? 0 : (3 *w/5.0) );                               //move label above thumb if horizontal, to the right if vertical         
                lblY = (horiz ? -(3*h/5.0) : 0);
                glTranslatef(trnX, trnY, 0);                                    //move to location for button based on slider progress
                this->buildLabel();
                KinUIComponent::drawBaseCmp(lblX, lblY);		
            glPopMatrix();
        }//draw

        //draw the rails the slider button operates on
        void KinUISlider::drawSliderBar(){
            float lenX, lenY, trX, trY, frm = KinSldrBrdr;
            lenX = (horiz ? sbDimL : sbDimTh);					//dimensions of slider based on orientation
            lenY = (horiz ? sbDimTh : sbDimL);

            trX = x + (horiz ? 0 : (abs(w - sbDimTh)/2));			//location of slider bar based on orientation
            trY = y + (horiz ? (abs(h - sbDimTh)/2) : 0);

            //glTranslatef(x,y,0);
            glPushMatrix();
            glTranslatef(trX, trY,0);
            glColor4f(0,0,0,1);
            glPolygonMode(GL_FRONT_AND_BACK,  GL_LINES);
            glLineWidth(1.0f);
            glBegin(GL_QUADS);
            glVertex2f(0,0);
            glVertex2f(lenX,0);
            glVertex2f(lenX,lenY);
            glVertex2f(0,lenY);
            glEnd();

            glColor4f(.4f,.4f,.4f,1);
            glPolygonMode(GL_FRONT_AND_BACK,  GL_LINES);
            glLineWidth(1.0f);
            glBegin(GL_QUADS);
            glVertex2f(frm,frm);
            glVertex2f(lenX-frm,frm);
            glVertex2f(lenX-frm,lenY-frm);
            glVertex2f(frm,lenY-frm);
            glEnd();
            glPopMatrix();
        }

        //handles kinect event - val gets value put into it
        void KinUISlider::getValueForEvent(int hand, int type, int drag, float& val){		       }			

        //if click in scroll bar (cmpObj == 2) then move "thumb"/indicator toward click location
        int KinUISlider::click(float _x, float _y, int cmpObj){	
            if(2 == cmpObj){                                                                                            //move thumb due to click happening in scroll bar
                slidePos = ((horiz) ? (_x - x) : (_y - y))/sbDimL ;						
                capSlidePos();
            }		
            float offset = (horiz ? -(w/2.0) + slidePos * sbDimL  : -(h/2.0) + (slidePos * sbDimL));					//displacement of thumb based on orientation and value
            stickX -= int(horiz ? offset : 0);                                                                  
            stickY -= int(horiz ? 0 : offset);
            return ID;
        }

        //drag thumb around
        int KinUISlider::drag(float _x, float _y){	
            float del = ((horiz) ? _x - clkX : _y - clkY) * UIobj_DragSens;					                            //sensitivity is modifiable
            slidePos += del/(1.0f*sbDimL);
            // cout<<"slider mod amt : "<< del/(1.0f * sbDimL)<<" del : "<<del<<" sbDimL "<<(sbDimL)<<" x,y : ("<<x<<", "<<y<<") _x,_y : ("<<_x<<", "<<_y<<") clkX,clkY : ("<<clkX<<", "<<clkY<<")"<<endl;
            capSlidePos();
            return ID;
        }

        std::ostream& operator<<(std::ostream& out, const KinUISlider& cmp){
            out<<static_cast<const KinUIComponent&>(cmp);                                                                        //start with base class
            out<<"Slider specific attribs : Min Val : "<<cmp.slideMin<<" Range : "<<cmp.slideRng<<" Current position (% of range) : "<<(cmp.slidePos * 100)<<"% "<<std::endl;
            out<<"\t# of subdivisions of range : "<<cmp.slideSubdiv<<" Slider track dimensions (Len,Thk) : ("<<cmp.sbDimL<<", "<<cmp.sbDimTh<<") Orientation : "<<(cmp.horiz ? "Horizontal " : "Vertical ")<<std::endl;
            out<<"\tIncreasing value dir : "<<(cmp.horiz ? (cmp.LToR ? "Left to Right" : "Right To Left") : (cmp.LToR ? "Down to Up" : "Up to Down"))<<std::endl;
            out<<"\tUse list for values : "<<cmp.useListVals<<endl;
             if(cmp.useListVals){
                out<<"\tDisplay List Values :";
                for(int i = 0; i < cmp.dispListVals.size(); ++i){   out<<(i == 0 ? " `" : ", `")<<cmp.dispListVals[i]<<"`";}
                out<<endl;
                out<<"\tReturned List Values : ";
                for(int i = 0; i < cmp.resListVals.size(); ++i){   out<<(i == 0 ? " `" : ", `")<<cmp.resListVals[i]<<"`";}
                out<<endl;
            }
           return out;
        }//op<<

    }//namespace kinect
}//namespace rtql8
