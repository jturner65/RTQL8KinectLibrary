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

#ifndef __KINUICRANK_H__
#define __KINUICRANK_H__
#include "KinUIComponent.h"
/*
	describes a UI component for the kinect to interact with 
    this is a crank-like device that responds to mouse drag or hand grab events acting in a circular motion
    useful for entering angle values or choosing preloaded values from a list (like a drop down list)
    the range of motion required and resultant values can both be configured upon instantiation
*/
using namespace std;

namespace rtql8 {
	namespace kinect{ 
		class KinUICrank : public KinUIComponent{
		public:
			KinUICrank():KinUIComponent(),
                crankMin(0), crankInitVal(0), crankRng(360), crankPos(0), crankPin(2),
                crankSubdiv(0),	ckDimL(0), ckDimTh(0), clRotate(true), useListVals(false),dispListVals(), resListVals(), isRadians(false)
                { init();}
			KinUICrank(int _x, int _y, int _w=0, int _h=0, float _cX = 0, float _cY=0, string _lbl="",string _msOverTxt="", string _ctsTxt=""):KinUIComponent(_x, _y, _w, h, _cX, _cY, _lbl, _msOverTxt,  _ctsTxt), 
                crankMin(0), crankInitVal(0), crankRng(360), crankPos(0), crankPin(2),
                crankSubdiv(0), ckDimL(0), ckDimTh(0), clRotate(true), useListVals(false),dispListVals(), resListVals(), isRadians(false)
                { init();}						//local init
			KinUICrank(const KinUICrank& _mB):KinUIComponent(_mB), 
                crankMin(_mB.crankMin), crankInitVal(_mB.crankInitVal), crankRng(_mB.crankRng), crankPos(_mB.crankPos), crankPin(_mB.crankPin),
                crankSubdiv (_mB.crankSubdiv),	ckDimL(_mB.ckDimL),	ckDimTh(_mB.ckDimTh), clRotate(_mB.clRotate), 
                useListVals(_mB.useListVals),dispListVals(_mB.dispListVals), resListVals(_mB.resListVals), isRadians(_mB.isRadians)
                {}
			~KinUICrank(void){}
	
			virtual int reInit();
			virtual void draw();
			void drawCrankBar(int rotVal, float hfC);
            void drawCrankHotSpot(int rotVal, float hfC);

            virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt){}
			virtual int click(float _x, float _y, int cmpObj);
			virtual int drag(float _x, float _y);
			
			virtual void getValueForEvent(int hand, int type, int drag, float& val);									
            virtual float getCurValue(){				return (crankMin + (crankPos * crankRng * (isRadians ? PI/180.0 : 1)));}		//angle value from crank

            void setCrankMin(float _cm){ crankMin = _cm;}
			void setCrankRng(float _cl){ crankRng = _cl;}
			void setCrankPos(float _cp){ crankPos = _cp;}
			void setCrankSubdiv(int _csv){ crankSubdiv = _csv;}	
			void setCkBarDimW(int _cx){ ckDimL = _cx; }
			void setCKBarDimH(int _cy){ ckDimTh = _cy; calcCrankPin();}
            //calculate the location of the crank pin that attaches the crank to the button.  use this as the origin of the crank
            void calcCrankPin(){    crankPin[0] = x + w/2.0;     crankPin[1] = y + h/2.0;    }

			void setCrankVals(float _cInit, float _cl,float _cp, int _csv,int _cx,int _cy){crankInitVal = _cInit; crankRng = _cl;	crankPos = _cp;	crankSubdiv = _csv;	ckDimL = _cx; ckDimTh = _cy; calcCrankPin();}
			void setIsClockWise(bool _cl){clRotate = _cl;}

            virtual void buildLabel(){                                              //modify labels on the fly
                std::string::size_type colLoc = label.find_first_of(":");
			    stringstream ss;
			    ss<<label.substr(0, colLoc)<<":"<<(int)(this->getCurValue())<<" deg";
			    label = ss.str();                                                           
            }//buildLabel

            void rotateLoc(int& _clX, int& _clY, int dir){                           //find rotated passed locatation (location relative to center of crank box) - dir needs to be 1 or -1
				float rotRad = dir * buildCrankRotVal() * PI/180.0f;
				Eigen::Vector3f vecClk = Eigen::Vector3f(_clX,_clY,0), vecC = Eigen::Vector3f(0,0,0); 				
				Eigen::Matrix3f t;
				t = Eigen::AngleAxis<float>(rotRad,Eigen::Vector3f(0,0,1));			//rotation is around -1 axis for bar, so rotate around z = 1 for click to see if it is inside bar - eigen internally converts type rotation matrix
				vecC = t * vecClk;													//rotate click to find click inside unrotated crank arm
                _clX = vecC(0); 
                _clY = vecC(1);
            }
            //clickable zone in mouse click coords - for use with hands, to make using objects easier
            //_xbrdr : border in x dir
            //_ybrdr : border in y dir
            virtual void setHotSpot(float _xBrdr, float _yBrdr){
                hsXBr = _xBrdr;
                hsYBr = _yBrdr;
                hsXL = x - _xBrdr; hsXH = x + w + _xBrdr; hsYL = y - _yBrdr; hsYH = y + h + _yBrdr;             //used in conjunction with calculated crank hotspot TODO
            }

            //true if inside button hs of crank
            bool isInsideHotSpotButton(int clckX, int clckY){    return (((clckX > hsXL) && (clckX < hsXH) && (clckY > hsYL) && (clckY < hsYH)) && (flags[UIobjIDX_Display]));}
            //true if inside crankbar hs of crank
            bool isInsideHotSpotCrank(int clckX, int clckY){	
				int _clX = clckX - (crankPin[0]), _clY = clckY - (crankPin[1]);
                rotateLoc(_clX, _clY,1);                                              //rotate passed click location to correspond to current bar angle
                return (((_clX > -hsXBr+ckDimTh/2.0f) && (_clX < hsXBr + ckDimL - ckDimTh/2.0f) && (_clY > -hsYBr+ckDimTh/2.0f) && (_clY < hsYBr - ckDimTh/2.0f)) && (flags[UIobjIDX_Display]));
            }//isInsideHotSpot

                //check if inside hotspot for button and for crank bar
            virtual int isInsideHotSpot(int clckX, int clckY){	return (isInsideHotSpotCrank(clckX, clckY) ? 2 : (isInsideHotSpotButton(clckX, clckY) ? 1 : 0));                          }//isInsideHotSpot

                //check if event happening inside slider/crank button and slider/crank bar
			virtual int isInside(int _clckX, int _clckY){
				int _clX = _clckX - (crankPin[0]), _clY = _clckY - (crankPin[1]);
                rotateLoc(_clX, _clY,1);                                                //rotate passed click location to correspond to current bar angle
				bool inButton = ((_clckX > x) && (_clckX < x + w) && (_clckY > y) && (_clckY < y + h));
                bool inCrank = ((_clX > 0) && (_clX < ckDimL) && (_clY > -ckDimTh/2.0f) && (_clY < ckDimTh/2.0f));
				int retVal = ((inButton || inCrank) ?					                //inside button portion or inside bar portion
							(inButton ? 1 : 2) : 0);			                        //inside button portion return 1 else return 2, if in neither, return 0;
				return retVal;
			}//isInside

            virtual void stickToClick(int& hndX, int& hndY){
                int tmpX = stickX, tmpY = stickY;
                rotateLoc(tmpX, tmpY,1);
                hndX = tmpX + x + w/2; 
                hndY = tmpY + y + h/2;
                //clog<<"after handx/y "<<hndX<<"|"<<hndY<<std::endl;
            }	

            //set appropriate "stick to" spot for hands, relative to x,y vals of UI obj - should snap and stick to this location on click and drag to snap to center of bar
            virtual void stickHandSpot(){
                int tmpLocX = ckDimL/2, tmpLocY = ckDimTh/2;
                rotateLoc(tmpLocX,tmpLocY,1);
                stickX = tmpLocX;
                stickY = -tmpLocY;
            }

			KinUICrank& operator=(KinUICrank other){
				std::swap(static_cast<KinUICrank>(*this), static_cast<KinUICrank>(other));
				return *this;
			}

            friend void swap(KinUICrank& _a, KinUICrank& _b){
				using std::swap;
				swap(static_cast<KinUIComponent>(_a), static_cast<KinUIComponent>(_b));
				swap(_a.crankMin, _b.crankMin);
				swap(_a.crankInitVal, _b.crankInitVal);
				swap(_a.crankRng, _b.crankRng);
				swap(_a.crankPos, _b.crankPos);
				swap(_a.crankPin, _b.crankPin);
				swap(_a.crankSubdiv, _b.crankSubdiv);
				swap(_a.ckDimL, _b.ckDimL);
				swap(_a.ckDimTh, _b.ckDimTh);
				swap(_a.clRotate, _b.clRotate);
				swap(_a.useListVals, _b.useListVals);
				swap(_a.isRadians, _b.isRadians);
				swap(_a.dispListVals, _b.dispListVals); 
				swap(_a.resListVals, _b.resListVals);
			}//swap

            friend std::ostream& operator<<(std::ostream& out, const KinUICrank& cmp);

		private : 
			virtual void init();
			//this will determine the appropriate display rotation for the crank. use for display and for checking click location (in degrees)
			int buildCrankRotVal(){	
				int rotVal = (clRotate ? -1 : 1);																//rotation of crank bar based on orientation and value
				rotVal *= ((int)((crankPos * crankRng) + crankInitVal - 90) % (int)crankRng);				    //set so that a value of 0 is at 12 oclock	
				return rotVal;
			}//buildCrankVerts

		private :	//variables					//along with inherited members from component, need to initialize the following
            float crankMin;                     //minimum crank value - default to 0
			float crankInitVal;					//where the crank starts at -> 0 == 12 oclock, values proceed clockwise.
			float crankRng;						//arc length of sliding capability 0 - 2pi or 0 - 360
			float crankPos;						//current position based on % of crankRng - 0.0 -> 1.0
            vector<float> crankPin;             //attach location of crank to button - should be center of button minus offset for thickness of crank
			int crankSubdiv;					//# of subdivisions of slider for discrete values - <= 0 means "continuous"
			int ckDimL, ckDimTh;				//dimensions of crank handle used to change crank value - length and width
			bool clRotate;						//orientation of crank = true is clockwise rotation increases, false is ccw increases
            bool useListVals;                   //whether or not to use an associated list of values (aka use as a drop down list box)
            bool isRadians;                     //whether or not we are using radians for our values
            vector<std::string> dispListVals;   //what values are displayed if this is used as a list box
            vector<std::string> resListVals;    //what values are returned as results if this is a list box
			
		};//KinUIComponent
	}//namespace kinect
}//namespace rtql8
#endif
