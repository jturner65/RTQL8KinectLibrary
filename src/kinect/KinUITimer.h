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

#ifndef __KINUITIMER_H__
#define __KINUITIMER_H__
#include "KinUISlider.h"
/*
  describes a UI component for the kinect to interact with 
  this is a slider device similar to a scroll bar. 
  the range of motion, orientation (horizontal or vertical) and result values can be specified upon instantiation
*/
using namespace std;

namespace rtql8 {
    namespace kinect{ 

        class KinUITimer : public KinUISlider{
        public:
            KinUITimer(): KinUISlider(), currVal(0), isIncr(true), isCntDn(true), isDone(false){init();}
            KinUITimer(double _currTime, bool _incr=true, bool _cntDn=true): KinUISlider(0,0), currVal(_currTime),  isIncr(_incr), isCntDn(_cntDn), isDone(false){init();}
            KinUITimer(int _x, int _y, int _w=0, int _h=0, float _sX = 0, float _sY=0, string _lbl="",
                       string _msOverTxt="", string _stsTxt="",double _currVal=0,  
                       bool _incr=true, bool _cntDn=true, bool _isDn=false ):
                KinUISlider(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt,  _stsTxt), currVal(_currVal), isIncr(_incr), isCntDn(_cntDn), isDone(_isDn){init();}			//local init		

            KinUITimer(const KinUITimer& _mB):KinUISlider(_mB), currVal(_mB.currVal), isIncr(_mB.isIncr), isCntDn(_mB.isCntDn), isDone(_mB.isDone){}
            ~KinUITimer(void){} 

            virtual int reInit();
            virtual void draw();
            void drawProgThumb(float tdelX, float tdelY, float delX, float delY);

            //timer only functions
            double getLastElapsed() const {       return lastElapsed;     }
            double getTotElapsed() const {       return totElapsed;     }
            double getLIToSecs( LARGE_INTEGER & L) {   return ((double)L.QuadPart /(double)freq.QuadPart) ;    }
            
            void setCurrVal(double _cv){currVal = _cv;}                     //what's the current progess value
            void setIsIncr(bool _isInc){isIncr = _isInc;}                   //whether or not the display for this object should increase or decrease (progress or amt/time left) 

            void startTimer() { 
                QueryPerformanceCounter(&timerStart) ; 
                timerLast = timerStart;
            }

            void calcTimeDiff(LARGE_INTEGER timeNow){
                LARGE_INTEGER time, tmpTimeNow;
                time.QuadPart = timeNow.QuadPart - timerStart.QuadPart;
                tmpTimeNow.QuadPart = timeNow.QuadPart - timerLast.QuadPart;
                timerLast.QuadPart = timeNow.QuadPart;
                totElapsed = getLIToSecs( time) ;
                lastElapsed = getLIToSecs( tmpTimeNow) ;
            }

            void updateTimerVals(){
                float dirMult = (LToR ? 1 : -1);                            //if left to right/up to down (the usual progress bar orientation) then add _dec amt, if right to left/down to up (timer countdown) then subtract       
                currVal += dirMult * lastElapsed;
                slidePos = (float)(currVal)/slideRng;                   
            }

            double elapsed(){                   
                LARGE_INTEGER timeNow;
                QueryPerformanceCounter(&timeNow) ;
                calcTimeDiff(timeNow);
                return lastElapsed;
            }
            
            void stopTimer() {
                QueryPerformanceCounter(&timerStop) ;
                calcTimeDiff(timerStop);
                updateTimerVals();
            }

            bool getIsDone() {            return isDone;        }

            bool advTimerVal(){                                                    
                //only use for progress bar, use timer implementations for timer
                // cout << "advTimerVal " << isCntDn << endl;
                bool res = false;         
                if(isCntDn){                                                            //if countdown timer only - stopwatch timer won't ever end
                    isDone = false;
                    elapsed();                                                          //calculate how much time has passed since last check
                    updateTimerVals();                                                  //update values 
                    if ((slidePos < 0) || (slidePos > 1)){
                        boundSlidePos();
                        isDone = true;
                        res = true;
                    } 
                }
                return res;
            }//advTimerVal

            void boundSlidePos(){                                           
                if ((slidePos < 0) || (slidePos > 1)){
                    currVal =  slideMin;
                    slidePos = 0;
                } 
            }//boundSlidePos

            void reset(){                                                               //initializes values         
                isDone = false;
                currVal =  slideMin + slideRng;
                slidePos =  1.0;
                startTimer();
                // cout << "reset: " << currVal << endl;
            }//

            void reset(float res){                                                      //initializes values         
                slideRng = ((res != slideRng) && (res > slideMin) ? res : slideRng);    //only change slideRng if reset value is appropriate value
                this->reset();
                startTimer();
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

            //timer counts up - gives time passed since started
            bool isStopWatch(){return !isCntDn;}
            bool isCntDnTimer(){return isCntDn;}

            void setIsCntDn(bool _cd){isCntDn = _cd;}

            KinUITimer& operator=(KinUITimer other){
                std::swap(static_cast<KinUITimer>(*this), static_cast<KinUITimer>(other));
                return *this;
            }
            friend void swap(KinUITimer& _a, KinUITimer& _b){
                using std::swap;
                swap(static_cast<KinUISlider>(_a), static_cast<KinUISlider>(_b));
                swap(_a.currVal, _b.currVal);
                swap(_a.isIncr, _b.isIncr);
                swap(_a.isCntDn, _b.isCntDn);
                swap(_a.isDone, _b.isDone);
            }//swap

            friend std::ostream& operator<<(std::ostream& out, const KinUITimer& cmp);
   
        private :	//functions
            virtual void init();
        
        private :	//variables					//along with inherited members from slider, need to initialize the following
            double currVal;                     //what's the current progess value
            bool isIncr;                        //whether or not the display for this object should increase or decrease
            bool isCntDn;                       //is a countdown timer (specific amount of time) - if false then is a stopwatch, automatically isIncr = true and ignore range;
            bool isDone;                        //timer/progress bar has expired/finished

            double lastElapsed;                 //since last check
            double totElapsed;                  //since start

            //timer only functions
            LARGE_INTEGER  timerStart;          //long long
            LARGE_INTEGER  timerLast;           //time since last timer check
            LARGE_INTEGER  timerStop;           //time at timer stop
            LARGE_INTEGER  freq;

        };//KinUISlider
    }//namespace kinect
}//namespace rtql8
#endif
