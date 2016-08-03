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

#ifndef __KINUIPRGBAR_H__
#define __KINUIPRGBAR_H__
#include "KinUISlider.h"
/*
  describes a UI component for the kinect to interact with 
  this is a slider device similar to a scroll bar. 
  the range of motion, orientation (horizontal or vertical) and result values can be specified upon instantiation
*/
using namespace std;

namespace rtql8 {
    namespace kinect{ 

    class KinUIPrgBar : public KinUISlider{
        public:
            KinUIPrgBar(): KinUISlider(), currVal(0), isIncr(true),isDone(false){init();}
            KinUIPrgBar(double _currProg, bool _incr=true): KinUISlider(0,0), currVal(_currProg),  isIncr(_incr),isDone(false){init();}
            KinUIPrgBar(int _x, int _y, int _w=0, int _h=0, float _sX = 0, float _sY=0, string _lbl="",
                        string _msOverTxt="", string _stsTxt="",double _currVal=0, int _cnt=0, 
                        bool _prgb=false, bool _incr=true, bool _isRng=false, bool _isDn=false ):
                        KinUISlider(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt,  _stsTxt), currVal(_currVal),  isIncr(_incr),isDone(_isDn){init();}			//local init		

            KinUIPrgBar(const KinUIPrgBar& _mB):KinUISlider(_mB), currVal(_mB.currVal), isDone(_mB.isDone){}
            ~KinUIPrgBar(void){} 

            virtual int reInit();
            virtual void draw();
            void drawProgress(float tdelX, float tdelY, float delX, float delY);
            bool getIsDone(){return isDone;}
           
            void setCurrVal(float _cv){currVal = _cv;}                                  //what's the current progess value
            void setIsIncr(bool _isInc){isIncr = _isInc;}                               //whether or not the display for this object should increase or decrease (amt progress made, or amt progress remaining)

            bool advPrgBarVal(float _dec){                                              //only use for progress bar, use timer implementations for timer
                bool res = false;         
                isDone = false;
                float dirMult = (LToR ? 1 : -1);                                        //if left to right/up to down (the usual progress bar orientation) then add _dec amt, if right to left/down to up (timer countdown) then subtract
                currVal += dirMult * _dec;
                slidePos = currVal/slideRng;                                            //ratio
                if ((slidePos < 0) || (slidePos > 1)){
                    boundSlidePos();
                    isDone = true;
                    res = true;
                } 
                return res;
            }//advPrgBarVal

            void boundSlidePos(){                                                       //make sure slide pos stays legal
                if (slidePos < 0) {
                    currVal =  slideMin;
                    slidePos =  0;
                } 
                if (slidePos > 1){
                    currVal =  slideMin + slideRng;
                    slidePos =  1;
                } 
            }//boundSlidePos

            void reset(){                                                               //initializes values         
                isDone = false;
                currVal =  slideMin ; 
                slidePos = 0 ;
            }//

            void reset(float res){                                                      //initializes values         
                slideRng = ((res != slideRng) && (res > slideMin) ? res : slideRng);    //only change slideRng if reset value is appropriate value
                this->reset();
            }//

            virtual void buildLabel(){                                                  //build label to include current timer value
                const int MAX_BUF = 256;
                char buf[MAX_BUF];
                std::string::size_type colLoc = label.find_first_of(":");       
                stringstream ss;
                sprintf(buf, "%.1f", (slideMin + (isIncr ? slidePos : 1-slidePos) * slideRng));                      //format for float                    
                ss<<label.substr(0, colLoc)<<":"<< buf;
                label = ss.str();
            }//buildLabel

            KinUIPrgBar& operator=(KinUIPrgBar other){
                std::swap(static_cast<KinUIPrgBar>(*this), static_cast<KinUIPrgBar>(other));
                return *this;
            }
            friend void swap(KinUIPrgBar& _a, KinUIPrgBar& _b){
                using std::swap;
                swap(static_cast<KinUISlider>(_a), static_cast<KinUISlider>(_b));
                swap(_a.currVal, _b.currVal);
                swap(_a.isIncr, _b.isIncr);
                swap(_a.isDone, _b.isDone);
            }//swap

            friend std::ostream& operator<<(std::ostream& out, const KinUIPrgBar& cmp);
   
        private :	//functions
            virtual void init();
        
        private :	//variables					//along with inherited members from slider, need to initialize the following
            float currVal;                     //what's the current progess value
            bool isIncr;                        //whether or not the display for this object should increase or decrease (progress or amt/time left)
            bool isDone;                        //timer/progress bar has expired/finished

        };//KinUISlider
    }//namespace kinect
}//namespace rtql8
#endif
