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
#include "KinUITimer.h"

using namespace std;

namespace rtql8{
    namespace kinect{

        int KinUITimer::reInit(){	
            KinUISlider::reInit();		            //parent reinit/init
            init();                                 //init this class's variables
            return 0;
        }

        void KinUITimer::init(){
             //initial values for various properties - timer/prgbar doesn't move from input
            flags[UIobjIDX_CanDrag] = false;
            flags[UIobjIDX_CanClick] = false;
            timerStart.QuadPart = 0;
            timerLast.QuadPart = 0;
            timerStop.QuadPart = 0;
            QueryPerformanceFrequency( &freq ) ;                                    //Retrieves the frequency of the high-resolution performance counter
        }

        //draw this component on the screen
        void KinUITimer::draw(){
            float trnX, trnY, lblX, lblY;								        //location of slider thumb, based on value of slider,with 0 putting at top/left, and 1 placing at bottom/right, and label above thumb
            glPushMatrix();
                drawSliderBar();
            glPopMatrix();
            glPushMatrix();
                trnX = (horiz ? -(w/2.0) + slidePos * sbDimL : 0);              
                trnY = (horiz ? 0 : -(h/2.0) + (slidePos * sbDimL) );
                lblX = (horiz ? 0 : (3 *w/5.0) );                               //move label above thumb if horizontal, to the right if vertical         
                lblY = (horiz ? -(3*h/5.0) : 0);
                this->buildLabel();
                glTranslatef(trnX, trnY, 0);                                    //move to location for button based on slider progress
                KinUIComponent::drawBaseCmp(lblX, lblY);		
            glPopMatrix();
        }//draw

        //draws stretched progbar thumb
        void KinUITimer::drawProgThumb(float tdelX, float tdelY, float delX, float delY){
            glPushMatrix();
            if(flags[UIobjIDX_MSDown]){	glColor4f(.5f-clr[0]/2.0f, .5f-clr[1]/2.0f, .5f-clr[2]/2.0f, clr[3]);} 
            else {			            glColor4f(clr[0]/2.0f,clr[1]/2.0f,clr[2]/2.0f,clr[3]);	}

            glTranslatef(x,y,0);
            glPolygonMode(GL_FRONT_AND_BACK,  GL_LINES);
            glLineWidth(1.0f);
            glBegin(GL_QUADS);
                glVertex2f(0,0);
                glVertex2f(tdelX+w,0);
                glVertex2f(tdelX+w,tdelY+h);
                glVertex2f(0,tdelY+h);
            glEnd();

            if(flags[UIobjIDX_MSDown]){	glColor4f(1.0f-clr[0], 1.0f - clr[1], 1.0f - clr[2],clr[3]);} 
            else {			            glColor4f(clr[0],clr[1],clr[2],clr[3]);	}
            glPolygonMode(GL_FRONT_AND_BACK,  GL_FILL);
            glBegin(GL_QUADS);
                glVertex2f(2,2);
                glVertex2f(tdelX+w-4,2);
                glVertex2f(tdelX+w-4,tdelY+h-4);
                glVertex2f(2,tdelY+h-4);
            glEnd();
            glPopMatrix();
            drawText(delX, delY, label);
        }//drawBaseCmp


        std::ostream& operator<<(std::ostream& out, const KinUITimer& cmp){
            out<<static_cast<const KinUISlider&>(cmp);                                                         //start with base class
            out<<"Timer specific attribs : Time left : "<<cmp.currVal;
            return out;
        }//op<<

    }//namespace kinect
}//namespace rtql8
