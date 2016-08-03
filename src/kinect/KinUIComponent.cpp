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
#include "KinUIComponent.h"
#include "KinectController.h"
#include "KinectHandler.h"

using namespace std;

namespace rtql8{
    namespace kinect{
        unsigned int KinUIComponent::ID_gen = 0;

        //specifically for being re-inited by external code for 
        int KinUIComponent::reInit(){
            //any external-specific functionality
            init();
            return 0;
        }

        void KinUIComponent::init(){
            //initial values for various properties
            flags[UIobjIDX_Display] = true;
            flags[UIobjIDX_CanClick] = true;
            flags[UIobjIDX_DispHS] = true;
            kinectController = NULL;
        }

        //draws button component of slider, crank and button 
        void KinUIComponent::drawBaseCmp(float delX, float delY){
            glPushMatrix();
            if(flags[UIobjIDX_MSDown]){	glColor4f(.5f-clr[0]/2.0f, .5f-clr[1]/2.0f, .5f-clr[2]/2.0f, clr[3]);} 
            else {			            glColor4f(clr[0]/2.0f,clr[1]/2.0f,clr[2]/2.0f,clr[3]);	}

            glTranslatef(x,y,0);
            glPolygonMode(GL_FRONT_AND_BACK,  GL_LINES);
            glLineWidth(1.0f);
            glBegin(GL_QUADS);
                glVertex2f(0,0);
                glVertex2f(w,0);
                glVertex2f(w,h);
                glVertex2f(0,h);
            glEnd();

            if(flags[UIobjIDX_MSDown]){	glColor4f(1.0f-clr[0], 1.0f - clr[1], 1.0f - clr[2],clr[3]);} 
            else {			            glColor4f(clr[0],clr[1],clr[2],clr[3]);	}
            glPolygonMode(GL_FRONT_AND_BACK,  GL_FILL);
            glBegin(GL_QUADS);
                glVertex2f(2,2);
                glVertex2f(w-4,2);
                glVertex2f(w-4,h-4);
                glVertex2f(2,h-4);
            glEnd();
            glPopMatrix();
            drawText(delX, delY, label);
        }//drawBaseCmp

        void KinUIComponent::drawText(float delX, float delY, string text, bool manScale, float scH){
            glPushMatrix();
                if(flags[UIobjIDX_OffsLbl]){    glTranslatef(x+delX,y+delY,0);}
                else {                          glTranslatef(x,y,0);}
                glColor4f(0,0,0,1);
                glLineWidth(2.0f);
                int lblLen = text.length();
                int chrStrkLen = 0;                                                         //length of glut stroke chars in "units" - use to dtermine size scaling
                float scVal;
                for(int i = 0; i < lblLen; ++i){                                chrStrkLen += glutStrokeWidth(GLUT_STROKE_ROMAN , text[i]);           }	
                if(manScale){                                                               //if passing manual scaling amount, use this, otherwise calculate scale amount
                    scVal = scH;
                    //glTranslatef(((w * scH)/2.0f), h/2.0f + (h * scH)/2.0f,0);				                       
                    //glScalef(scH, -scH, 1);
                }                                
                else{                                                                           //if not manual scaling - calculate appropriate scaling to keep text within ui component boundaries (w x h)               
                    scVal = min(max(.9f * w / chrStrkLen, UIobj_MinFontScale), UIobj_MaxFontScale);                      
                }
                glTranslatef((w - (chrStrkLen * scVal))/2.0f, (h + (h * scVal))/2.0f,0);	 //cap letters are approx 100 pxls high before scaling                   
                glScalef(scVal,-scVal,1);
                
                //display what object is causing event- debug only
                if(this->flags[UIobjIDX_DispDebug]){
                    string debugDisp(this->getChCtrl());                    
                    for(int i = 0; i < debugDisp.length(); ++i){                glutStrokeCharacter(GLUT_STROKE_ROMAN , debugDisp[i]);       }
                }
                for(int i = 0; i < lblLen; ++i){                                glutStrokeCharacter(GLUT_STROKE_ROMAN , text[i]);           }	
            glPopMatrix();

        }//drawTex

        void KinUIComponent::drawHotSpot() {

            if (kinectController != NULL) {
                KinectHandler* handler = kinectController->getKinectHandler();
                if (handler->kinValid() == false) {
                    return;
                }
            }
            glPushMatrix();
            glPolygonMode(GL_FRONT_AND_BACK,  GL_FILL);
            if(flags[UIobjIDX_MSDown]){	glColor4f(1.0f-clr[0], 1.0f - clr[1], 1.0f - clr[2],clr[3]/3.0f);} 
            else {						glColor4f(clr[0],clr[1],clr[2],clr[3]/3.0f);	}
            glLineWidth(1.0f);
            // glBegin(GL_QUADS);
            // glVertex2f(hsXL,hsYL);
            // glVertex2f(hsXH,hsYL);
            // glVertex2f(hsXH,hsYH);
            // glVertex2f(hsXL,hsYH);
            // glEnd();
            glPopMatrix();
        }//drawHotSpot

        //handles kinect event
        int KinUIComponent::click(float _x, float _y, int cmpObj){	
            int res = -1;
            if((_x != -1) && (_y != -1)){
                stickX = _x - x;                                                    //click location relative to center of object - stick control to this relative location as ui component moves
                stickY = _y - y;
                res = this->click(_x, _y, cmpObj);								    //object specific handler - add 1 for obj 0
                this->clkX = _x; this->clkY = _y;									//set click location, for next round
            }
            //else {   }
            return res;
        }//click

        int KinUIComponent::drag(float _x, float _y){	
            int res = this->drag(_x, _y);							                //object specific handler
            this->clkX = _x; this->clkY = _y;							            //set click location, for next round
            return res;
        }
		
        std::ostream& operator<<(std::ostream& out, const KinUIComponent& cmp){
            out<<"ID : "<<cmp.ID<<" label : "<<cmp.label<<" onClick : "<<cmp.onclick<<endl;
            out<<"location : ("<<cmp.x<<", "<<cmp.y<<") dim : ("<<cmp.w<<", "<<cmp.h<<")"<<std::endl;
            out<<"Color : (r: "<<cmp.clr[0]<<" g: "<<cmp.clr[1]<<" b: "<<cmp.clr[2]<<" a: "<<cmp.clr[3]<<") "<<std::endl;
            out<<"HotSpot : x (L,H) : ("<<cmp.hsXL<<", "<<cmp.hsXH<<") y (L,H) : ("<<cmp.hsYL<<", "<<cmp.hsYH<<") border (x,y) : ("<<cmp.hsXBr<<", "<<cmp.hsYBr<<") "<<std::endl;
            out<<"Flags : Display : "<<cmp.flags[UIobjIDX_Display]<<" Display Hot Spot : "<<cmp.flags[UIobjIDX_DispHS]<<" Can Click : "<<cmp.flags[UIobjIDX_CanClick]<<" Can Drag : "<<cmp.flags[UIobjIDX_CanDrag]<<endl;
            return out;
        }


    }//namespace kinect
}//namespace rtql8
