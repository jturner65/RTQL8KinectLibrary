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

#ifndef __KINUISLIDER_H__
#define __KINUISLIDER_H__
#include "KinUIComponent.h"
/*
  describes a UI component for the kinect to interact with 
  this is a slider device similar to a scroll bar. 
  the range of motion, orientation (horizontal or vertical) and result values can be specified upon instantiation
*/
using namespace std;

namespace rtql8 {
    namespace kinect{ 
        class KinUISlider : public KinUIComponent{
        public:
            KinUISlider():KinUIComponent(),slideMin(0),  slideRng(0), slidePos(0),slideSubdiv(0), sbDimL(0), sbDimTh(0), horiz(true), LToR(true),useListVals(false), dispListVals(0), resListVals(0) {init();}
            KinUISlider(int _x, int _y, int _w=0, int _h=0, float _sX = 0, float _sY=0, string _lbl="",string _msOverTxt="", string _stsTxt=""):
                KinUIComponent(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt,  _stsTxt), slideMin(0), slideRng(0), slidePos(0),slideSubdiv(0),	sbDimL(0), sbDimTh(0), horiz(true),LToR(true),useListVals(false),dispListVals(0), resListVals(0){init();}							//local init		

            KinUISlider(const KinUISlider& _mB):
                KinUIComponent(_mB), slideMin(_mB.slideMin), slideRng(_mB.slideRng), slidePos(_mB.slidePos), slideSubdiv ( _mB.slideSubdiv), sbDimL(_mB.sbDimL),sbDimTh(_mB.sbDimTh), horiz(_mB.horiz),
                    LToR(_mB.LToR), useListVals(_mB.useListVals),dispListVals(_mB.dispListVals), resListVals(_mB.resListVals){}
            ~KinUISlider(void){}
	
            virtual int reInit();
            virtual void draw();
            void drawSliderBar();

            virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt){}
            virtual int click(float _x, float _y, int cmpObj);
            virtual int drag(float _x, float _y);
			
            virtual void getValueForEvent(int hand, int type, int drag, float& val);									
            virtual float getCurValue(){return slideMin + (LToR ? slidePos : 1-slidePos) * slideRng;}

            void setSlideMin(float _sm){ slideMin = _sm;}
            void setSlideRng(float _sl){ slideRng = _sl;}
            void setSlidePos(float _sp){ slidePos = _sp;}
            void setSlideSubdiv(int _ssv){ slideSubdiv = _ssv;}	
            void setSbDimW(int _sx){ sbDimL = _sx;}
            void setSbDimH(int _sy){ sbDimTh = _sy;}

            float getSliderPos() { return slidePos; }

            void setSlideVals(float _sl,float _sp,int _ssv,int _sx,int _sy){slideRng = _sl;	slidePos = _sp;	slideSubdiv = _ssv;	sbDimL = _sx; sbDimTh = _sy;
                buildHotSpot();
            }

            void setIsHoriz(bool _hz){horiz = _hz;}
            void setIsLToR(bool _lt){LToR = _lt;}

            virtual void buildLabel(){                                          //build label to include current slider value
                const int MAX_BUF = 256;
                char buf[MAX_BUF];
                std::string::size_type colLoc = label.find_first_of(":");       
                stringstream ss;
                sprintf(buf, "%.2f", this->getCurValue());                      
                ss<<label.substr(0, colLoc)<<":"<< buf;
                label = ss.str();
            }//buildLabel

            virtual void setHotSpot(float _xBrdr, float _yBrdr){
                float lenX = (horiz ? sbDimL : sbDimTh),						//dimensions of slider based on orientation
                    lenY = (horiz ? sbDimTh : sbDimL),
                    trX = x + (horiz ? 0 : (abs(w - sbDimTh)/2)),			    //location of slider bar based on orientation
                    trY = y + (horiz ? (abs(h - sbDimTh)/2) : 0);			
                hsXBr = _xBrdr;
                hsYBr = _yBrdr;
                hsXL = trX - _xBrdr; //x - _xBrdr; 
                hsXH = trX + lenX + _xBrdr;//x + w + _xBrdr; 
                hsYL = trY - _yBrdr;//y - _yBrdr; 
                hsYH = trY + lenY + _yBrdr;//y + h + _yBrdr;
            }//setHotSpot

            virtual void buildHotSpot() {
                float lenX = (horiz ? sbDimL : sbDimTh),						//dimensions of slider based on orientation
                    lenY = (horiz ? sbDimTh : sbDimL),
                    trX = x + (horiz ? 0 : (abs(w - sbDimTh)/2)),			    //location of slider bar based on orientation
                    trY = y + (horiz ? (abs(h - sbDimTh)/2) : 0);
                hsXL = trX - hsXBr; //x - _xBrdr; 
                hsXH = trX + lenX + hsXBr;//x + w + _xBrdr; 
                hsYL = trY - hsYBr;//y - _yBrdr; 
                hsYH = trY + lenY + hsYBr;//y + h + _yBrdr;
            }

            //check if query location is inside control's hotspot - used with hands to allow them to snap to center of control object
            virtual int isInsideHotSpot(int clckX, int clckY){	return (((clckX > hsXL) && (clckX < hsXH) && (clckY > hsYL) && (clckY < hsYH)) ? 2 : 0);}

            virtual int isInside(int clckX, int clckY){
                int inThumb = isInsideThumb(clckX, clckY), inTrack = isInsideTrack(clckX, clckY);
                return (inThumb || inTrack) ? (inThumb ? 1 : 2) : 0;
            }//isInside
            //checks if in thumb (button part) of slider
            bool isInsideThumb(int clckX, int clckY){
                float offset = (horiz ? -(w/2.0) + slidePos * sbDimL  : -(h/2.0) + (slidePos * sbDimL));					//displacement of thumb based on orientation and value	
                return (horiz ? 
                        ((clckX > x + offset) && (clckX < x + w + offset) && (clckY > y) && (clckY < y + h)) :				//displace "hotspot" in x if horizontal
                        ((clckX > x ) && (clckX < x + w) && (clckY > y + offset) && (clckY < y + h + offset)));				//displace in y if vertical
            }//isInsideThumb

            //checks if in track part of slider
            bool isInsideTrack(int clckX, int clckY){
                float lenX = (horiz ? sbDimL : sbDimTh),						//dimensions of slider based on orientation
                    lenY = (horiz ? sbDimTh : sbDimL),
                    trX = x + (horiz ? 0 : (abs(w - sbDimTh)/2)),			//location of slider bar based on orientation
                    trY = y + (horiz ? (abs(h - sbDimTh)/2) : 0);			
                return ((clckX >= trX) && (clckX <= trX+lenX)) && ((clckY >= trY) && (clckY <= trY+lenY));		
            }//isInsideTrack

            virtual void stickToClick(int& hndX, int& hndY){
                float offset = (horiz ? -(w/2.0) + slidePos * sbDimL  : -(h/2.0) + (slidePos * sbDimL));					//displacement of thumb based on orientation and value
                hndX = stickX + x + int(horiz ? offset : 0);
                hndY = stickY + y + int(horiz ? 0 : offset);
            }	

            KinUISlider& operator=(KinUISlider other){
                std::swap(static_cast<KinUISlider>(*this), static_cast<KinUISlider>(other));
                return *this;
            }
            friend void swap(KinUISlider& _a, KinUISlider& _b){
                using std::swap;
                swap(static_cast<KinUIComponent>(_a), static_cast<KinUIComponent>(_b));
                swap(_a.slideMin, _b.slideMin);
                swap(_a.slideRng, _b.slideRng);
                swap(_a.slidePos, _b.slidePos);
                swap(_a.slideSubdiv, _b.slideSubdiv);
                swap(_a.sbDimL, _b.sbDimL);
                swap(_a.sbDimTh, _b.sbDimTh);
                swap(_a.horiz, _b.horiz);
                swap(_a.LToR, _b.LToR);
                swap(_a.useListVals, _b.useListVals);
				swap(_a.dispListVals, _b.dispListVals); 
				swap(_a.resListVals, _b.resListVals);
            }//swap

            friend std::ostream& operator<<(std::ostream& out, const KinUISlider& cmp);

        protected :	//functions
            virtual void init();
            void capSlidePos(){		if(slidePos >= 1) {slidePos = 1;}	else if(slidePos <= 0) { slidePos = 0;}	}				//bound sliderpos values to between 0 and 1

        protected :	//variables					//along with inherited members from component, need to initialize the following
            float slideMin;                     //min value for slider
            float slideRng;						//length of sliding capability == max value (if 0 is min)
            float slidePos;						//0-1 position along slider -> 0 at top/left, 1 at bottom/right : multiplied with slideRng to be used as value of slider
            int slideSubdiv;					//# of subdivisions of slider for discrete values - <= 0 means "continuous"
            int sbDimL, sbDimTh;				//dimensions of sliderbar track (where slider thumb moves) - length and thickness
            bool horiz;							//orientation of slider - true is horizontal, false is vertical
            bool LToR;                          //whether the values INCREASE left to right(horiz)/up to down(vert) or right to left/down to up
            bool useListVals;                   //use list values instead of numerics for this slider (like a list box)
            vector<std::string> dispListVals;   //what values are displayed if this is used as a list box
            vector<std::string> resListVals;    //what values are returned as results if this is a list box
			
        };//KinUIComponent
    }//namespace kinect
}//namespace rtql8
#endif
