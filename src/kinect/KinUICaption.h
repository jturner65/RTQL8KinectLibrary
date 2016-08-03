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

#ifndef __KINUICAPTION_H__
#define __KINUICAPTION_H__
#include "KinUIComponent.h"
/*
  describes a UI component for the kinect to interact with : 
  this is a simple 2d button that should respond to mouse click, as well as hand click or grab events
*/
using namespace std;

namespace rtql8 {
    namespace kinect{ 
        class KinUICaption : public KinUIComponent{
        public:
            KinUICaption():KinUIComponent(){init();}
            KinUICaption(int _x, int _y, int _w=0, int _h=0, float _sX = 0, float _sY=0, string _lbl="",string _msOverTxt="", string _stsTxt=""):
                KinUIComponent(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt,  _stsTxt)	{	init();		}				//local init
            KinUICaption(const KinUICaption& _mB):KinUIComponent(_mB){}//should be copyswap
            ~KinUICaption(void){}

            virtual int reInit();																									//specifically designed to be called manually
            virtual void draw();
            virtual bool isClicked() {   bool retVal = flags[UIobjIDX_Click]; flags[UIobjIDX_Click] = false;    return retVal;       }
            virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt){}
            virtual int click(float _x, float _y, int cmpObj);
            virtual int drag(float _x, float _y){return -1;}

            virtual void getValueForEvent(int hand, int type, int drag, float& val);									
            virtual float getCurValue(){return 1;}							//button will just provide single value with default event
            virtual void buildLabel(){     }                                //modify labels on the fly
            KinUICaption& operator=(KinUICaption other){
                std::swap(static_cast<KinUICaption>(*this), static_cast<KinUICaption>(other));
                return *this;
            }

            friend void swap(KinUICaption& _a, KinUICaption& _b){
                using std::swap;
                swap(static_cast<KinUIComponent>(_a), static_cast<KinUIComponent>(_b));
            }//swap

            friend std::ostream& operator<<(std::ostream& out, const KinUICaption& cmp);

        private : //functions
            virtual void init();

        private : //variables
        };//KinUICaption
    }//namespace kinect
}//namespace rtql8
#endif
