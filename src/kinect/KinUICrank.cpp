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
#include "KinUICrank.h"

using namespace std;

namespace rtql8{
	namespace kinect{

		int KinUICrank::reInit(){
			KinUIComponent::reInit();		//parent reinit/init
			init();
			return 0;
		}

		void KinUICrank::init(){
			flags[UIobjIDX_CanDrag] = true;
            calcCrankPin();
		}
			//draw this component on the screen
		void KinUICrank::draw(){
            float lblX,lblY;
            int rotVal = buildCrankRotVal();
            float hfC = ckDimTh/2.0f;				

			glPushMatrix();
            if(flags[UIobjIDX_DispHS]){   this->drawHotSpot(); this->drawCrankHotSpot(rotVal,hfC);       }
			    drawCrankBar(rotVal,hfC);
                bool crankOnRight = ((clRotate && this->getCurValue() < 180 *(isRadians ? PI/180.0 : 1)) || (!clRotate && this->getCurValue() > 180 * (isRadians ? PI/180.0 : 1)));
                lblX = (crankOnRight ? (x < w ?  (3 *w/5.0) : -(3 *w/5.0)) : ( x > (winX - 2*w) ? -(3 *w/5.0) : (3 *w/5.0)));   //check if close to left edge
                lblY = (y < h ?  (3*h/5.0) : -(3*h/5.0));       //check if close to top
                this->buildLabel();
			    KinUIComponent::drawBaseCmp(lblX,lblY);		
			glPopMatrix();
		}//draw
			//draw the rails the slider button operates on
		void KinUICrank::drawCrankBar(int rotVal, float hfC){
			glPushMatrix();
				glTranslatef(crankPin[0], crankPin[1],0);
				glRotatef(rotVal,0,0,-1);
				glColor4f(0,0,0,1);
				glPolygonMode(GL_FRONT_AND_BACK,  GL_LINES);
				glLineWidth(1.0f);
				glBegin(GL_QUADS);
					glVertex2f(-hfC,-hfC);
					glVertex2f(ckDimL-hfC,-hfC);
					glVertex2f(ckDimL-hfC,ckDimTh-hfC);
					glVertex2f(-hfC,ckDimTh-hfC);
				glEnd();
				glTranslatef(0,0,.01f);

				glColor4f(.4f,.4f,.4f,1);
				glPolygonMode(GL_FRONT_AND_BACK,  GL_LINES);
				glLineWidth(1.0f);
				glBegin(GL_QUADS);
					glVertex2f(KinSldrBrdr-hfC,KinSldrBrdr-hfC);
					glVertex2f(ckDimL-KinSldrBrdr-hfC,KinSldrBrdr-hfC);
					glVertex2f(ckDimL-KinSldrBrdr-hfC,ckDimTh-KinSldrBrdr-hfC);
					glVertex2f(KinSldrBrdr-hfC,ckDimTh-KinSldrBrdr-hfC);
				glEnd();
			glPopMatrix();
		}//drawCrankBar		

        //draw hotspot for crankbar
		void KinUICrank::drawCrankHotSpot(int rotVal, float hfC){
			glPushMatrix();
				glTranslatef(crankPin[0], crankPin[1],0);
				glRotatef(rotVal,0,0,-1);
                glPolygonMode(GL_FRONT_AND_BACK,  GL_FILL);
                if(flags[UIobjIDX_MSDown]){	glColor4f(1.0f-clr[0], 1.0f - clr[1], 1.0f - clr[2],clr[3]/3.0f);} 
                else {						glColor4f(clr[0],clr[1],clr[2],clr[3]/3.0f);	}
				glLineWidth(1.0f);
				glBegin(GL_QUADS);
					glVertex2f(-hsXBr-hfC,hsYBr-hfC);
					glVertex2f(ckDimL+hsXBr-hfC,hsYBr-hfC);
					glVertex2f(ckDimL+hsXBr-hfC,ckDimTh-hsYBr-hfC);
					glVertex2f(-hsXBr-hfC,ckDimTh-hsYBr-hfC);
				glEnd();
			glPopMatrix();
		}//drawCrankBar

			//handles kinect event - val gets value put into it
		void KinUICrank::getValueForEvent(int hand, int type, int drag, float& val){	}			

		int KinUICrank::click(float _x, float _y, int cmpObj){
            calcCrankPin();
            stickX -= w/2;                                                                  
            stickY -= h/2;
            int tmpX = int(stickX), tmpY = int(stickY);
            rotateLoc(tmpX, tmpY,-1);
            stickX = tmpX;
            stickY = tmpY;
            //clog<<"crank stick location : ID "<<this->ID<<" : "<<stickX<<"|"<<stickY<<" x,y of obj : "<<x<<"|"<<y<<std::endl;         
            return ID;
        }

		int KinUICrank::drag(float _x, float _y){	
			Eigen::Vector3f clkCrnk = Eigen::Vector3f( _x - x, _y - y, 0),							//vector from center to click position
							clkCmp = Eigen::Vector3f(clkX - x, clkY - y, 0);						//vec from center to old click 
			float val = clkCmp.normalized().cross(clkCrnk.normalized())(2);
			//crankPos += val/(.02f*crankRng);
			crankPos += val/(UIobj_CrankSens*ckDimL);
            //cout<<"crank mod amt : "<< val/(UIobj_CrankSens*ckDimL)<<" val : "<<val<<" sens * ckDimL "<<(UIobj_CrankSens*ckDimL)<<" x,y : ("<<x<<", "<<y<<") _x,_y : ("<<_x<<", "<<_y<<") clkX,clkY : ("<<clkX<<", "<<clkY<<")"<<endl;
			if(crankPos >= 1) {crankPos = 1;}
			else if(crankPos <= 0) { crankPos = 0;}
			return ID;
		}

        std::ostream& operator<<(std::ostream& out, const KinUICrank& cmp){
            out<<static_cast<const KinUIComponent&>(cmp);                                                         //start with base class
            out<<"Crank specific attribs : Min Val : "<<cmp.crankMin<<" Range : "<<cmp.crankRng<<" Current position (% of range) : "<<(cmp.crankPos * 100)<<"% "<<" Crank Pin loc : ("<<cmp.crankPin[0]<<", "<<cmp.crankPin[1]<<")"<<std::endl;
            out<<"\t# of subdivisions of range : "<<cmp.crankSubdiv<<" Crank Handle dimensions (Len,Thk) : ("<<cmp.ckDimL<<", "<<cmp.ckDimTh<<") Rotation : "<<(cmp.clRotate ? "ClockWise " : "Counter Clockwise ")<<std::endl;
            out<<"\tUse list for values : "<<cmp.useListVals<<" Use Radians as angle measurement : "<<cmp.isRadians<<endl;
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
