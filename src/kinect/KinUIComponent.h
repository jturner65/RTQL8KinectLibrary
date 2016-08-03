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

#ifndef __KINUICOMPONENT_H__
#define __KINUICOMPONENT_H__

/*
  describes the base code for a UI component for the kinect to interact with - a button, a slider, etc
  uses upper left corner as 0,0
*/
using namespace std;

namespace rtql8 {
    namespace kinect{ 
        class KinectController;
        
        class KinUIComponent{
        public:
            //KinUIComponent():ID(-1),flags(UIobj_numFlags,false),x(0),y(0),w(0),h(0),clkX(0), clkY(0), stickX(0),stickY(0), UIvalue(), label(""),msOverTxt(""), stsTxt(""), clr(4), onclick(""), srcOfEvnt(-1){init();}
            KinUIComponent():ID(ID_gen++),flags(UIobj_numFlags,false),x(0),y(0),w(0),h(0),clkX(0), clkY(0), stickX(0),stickY(0), UIvalue(UIobj_numVals), label(""),msOverTxt(""), stsTxt(""), clr(4), onclick(""), srcOfEvnt(-1){init();}
            KinUIComponent(int _x, int _y, int _w=0, int _h=0, float _sX = 0, float _sY=0, string _lbl="",string _msOverTxt="", string _stsTxt=""):
                ID(ID_gen++),
                flags(UIobj_numFlags,false),
                x(_x),y(_y),w(_w),h(_h),
                clkX(0), clkY(0), 
                stickX(_sX),stickY(_sY), UIvalue(UIobj_numVals), 
                label(_lbl), msOverTxt(_msOverTxt), stsTxt(_stsTxt),clr(4), onclick(""), srcOfEvnt(-1){	
                    init();	}

            KinUIComponent::KinUIComponent(const KinUIComponent& _mB): ID(_mB.ID),flags(_mB.flags), x(_mB.x),y(_mB.y),w(_mB.w),h(_mB.h),stickX(_mB.stickX),stickY(_mB.stickY), UIvalue(_mB.UIvalue),  
                label(_mB.label),msOverTxt(_mB.msOverTxt), stsTxt(_mB.stsTxt), clr(_mB.clr), onclick(_mB.onclick), srcOfEvnt(_mB.srcOfEvnt) {}
            virtual ~KinUIComponent(void){}

            virtual int reInit();																									//specifically designed to be called manually
            virtual void draw(){glDisable(GL_LIGHTING);	this->draw();	glEnable(GL_LIGHTING);}
            void drawBaseCmp(float delX=0, float delY=0);           //delx, dely : displacement for label from thumb

            void drawHotSpot();

            //passed offset, text, and (optional) scale amount (to keep text scaled to specific value
            void drawText(float delX, float delY, string text, bool manScale=false, float scH=1);

            virtual int click(float _x, float _y, int cmpObj);
            virtual int drag(float _x, float _y);

            //returns value of control based on event - l/r hand, click down, up or drag - overridden in child classes  TODO
            virtual void getValueForEvent(int hand, int type, int drag, float& val){}									
            virtual float getCurValue(){return 0;}
			
            bool checkFlags(int idx){	           return this->flags[idx];	    }

                //check if this object is currently being dragged by the passed controller (either mouse, left hand or right hand)
            bool checkDragByThisCntrl(int cntrl){       return (this->flags[UIobjIDX_MSDrag] && (cntrl== this->srcOfEvnt));}
            bool canDrag(){                             return this->flags[UIobjIDX_CanDrag];}

            void setOnClick(string& oc) { onclick = oc; }
            string getOnClick() { return onclick; }

            //configuration values
            void setLoc(float _x, float _y){
                x = _x; y = _y;
                buildHotSpot();
            }
            void setDim(float _w, float _h){
                w = _w; h = _h;
                buildHotSpot();
            }
            void setColor(float r, float g, float b, float a){clr[0] = r; clr[1] = g; clr[2] = b; clr[3] = a;}
            void setDisplayHotSpot(bool _showHS){flags[UIobjIDX_DispHS] = _showHS;}
            void setStick(float _x, float _y){stickX = _x; stickY = _y;}
            void setLabel(string _txt){label = _txt;}
            void setMSOverTxt(string _txt){msOverTxt = _txt;}
            void setStsTxt(string _txt){stsTxt = _txt;}
            void setUIProps(bool _canClick, bool _canDrag){flags[UIobjIDX_CanClick] = _canClick; flags[UIobjIDX_CanDrag] = _canDrag;}

            void setSrcOfEvnt(int _src){srcOfEvnt = _src;}

            int getSrcOfEvnt(){return srcOfEvnt;}
            std::string getLabel() { return label; }
            std::string getMouseOverTxt() { return ("" == msOverTxt ?  label : msOverTxt); }
            std::string getStatusTxt() { return ("" == stsTxt ?  label : stsTxt); }
            int getID(){return ID;}

            virtual void buildLabel(){ this->buildLabel();}                                                                             //modify labels on the fly

            virtual bool isClicked() {return flags[UIobjIDX_MSDown];}
            virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt){}                                                          //clear object-specific code

            void clearClicked(float _x, float _y, int _srcOfEvnt){
                this->clearObjClicked(_x,_y,_srcOfEvnt);                                                                            //call object-specific code
                if(_srcOfEvnt == this->srcOfEvnt){                                                                                      //if this object caused click, then clear click
                    if(flags[UIobjIDX_MSDown]){flags[UIobjIDX_Click] = true;  } else {  flags[UIobjIDX_Click] = false;   }              //click event when mouse down cleared
                    flags[UIobjIDX_MSDown] = false;    flags[UIobjIDX_MSDrag] = false; 
                    stickX = -1;   stickY = -1; this->srcOfEvnt = -1;
                }
            }//clearClicked

            //set upon initial click in an object
            //_x,_y location of click
            //cmpobj - component object clicked on (when objects are compound objects like sliders or crank bars, this is what part was clicked in) 0 : not clicked in, 1 : button part, 2 : sliderbar/crank bar part
            //_srcOfEvnt - whether this was from the mouse or the left or right hands
            void setClicked(float _x, float _y, int cmpObj, int _srcOfEvnt){
                if(flags[UIobjIDX_CanClick]) {
                    flags[UIobjIDX_MSDown] = true;
                    this->srcOfEvnt = _srcOfEvnt;
                    if(flags[UIobjIDX_CanDrag]){       flags[UIobjIDX_MSDrag] = true;}                                              //if this can be dragged, and it isn't already dragging, then set dragged to passed bool                        
                    KinUIComponent::click(_x,_y, cmpObj);	                                                                        //object specific click down handling
                }
            }//setClicked

            //return -1 if fail
            int setDragged(bool _drag, float _x, float _y, int _srcOfDrag){
                if((flags[UIobjIDX_CanDrag]) && (flags[UIobjIDX_MSDrag]) && (_srcOfDrag == this->srcOfEvnt)) {				        //if it can drag and it is dragging by querying event source, then drag
                    return KinUIComponent::drag(_x,_y);		                                                                        //id of object being dragged if success
                }
                return -1;
            }						
            //set whether or not this object should be displayed
            void setDisplay(bool _disp){flags[UIobjIDX_Display] = _disp;}

            //clickable zone in mouse click coords - for use with hands, to make using objects easier
            //_xbrdr : border in x dir
            //_ybrdr : border in y dir
            virtual void setHotSpot(float _xBrdr, float _yBrdr){
                hsXBr = _xBrdr;
                hsYBr = _yBrdr;
                buildHotSpot();
            }

            virtual void buildHotSpot() {
                hsXL = x - hsXBr; hsXH = x + w + hsXBr;
                hsYL = y - hsYBr; hsYH = y + h + hsYBr;
            }
            
            //stick to center of object - put center values into _hx and _hy
            virtual void stickToClick(int& hndX, int& hndY){
                hndX = stickX + x; 
                hndY = stickY + y;
            }

            //set appropriate "stick to" spot for hands, relative to x,y vals of UI obj - should snap and stick to this location on click and drag - override for crank, to snap to center of bar
            virtual void stickHandSpot(){
                stickX = (w/2);
                stickY = (h/2);
            }
            
            //force hand to specific spot within ui object, for drag functionality
            void stickHandToClick(int& hndX, int& hndY){
                this->stickHandSpot();
                this->stickToClick(hndX, hndY);
            }

            void setWinSize(int _wx, int _wy){winX = _wx; winY = _wy;}

            //check if query location is inside control's hotspot - used with hands to allow them to stick to center of control object
            virtual int isInsideHotSpot(int clckX, int clckY){
                return (((clckX > hsXL) && (clckX < hsXH) &&
                         (clckY > hsYL) && (clckY < hsYH)) &&
                        (flags[UIobjIDX_Display])
                        && (flags[UIobjIDX_CanClick]) ? 1 : 0);}
            //check if query location is inside control's boundaries - returns 0 or 1 by default, will return other values if inside parts of multipart ui constructs		
            virtual int isInside(int clckX, int clckY){	return (((clckX > x) && (clckX < x + w) && (clckY > y) && (clckY < y + h)) && (flags[UIobjIDX_Display]) && (flags[UIobjIDX_CanClick]) ? 1 : 0);	}

            //returns string representing object causing current event - for debug
            string getChCtrl(){ 
                switch (srcOfEvnt) {
                    case KC_MOUSE   : {   return "M:";}
                    case KC_LEFT    : {   return "L:";}
                    case KC_RIGHT   : {   return "R:";}
                    default         : {   return "";}
                }
                return "";
            }//getChCtrl

            //copyswap
            KinUIComponent& operator=(KinUIComponent other){		//other is already copy
                std::swap(static_cast<KinUIComponent>(*this), static_cast<KinUIComponent>(other));
                return *this;
            }

            friend void swap(KinUIComponent& _a, KinUIComponent& _b){
                using std::swap;
                swap(_a.ID, _b.ID);
                swap(_a.x, _b.x);
                swap(_a.y, _b.y);
                swap(_a.w, _b.w);
                swap(_a.h, _b.h);
                swap(_a.hsXL,_b.hsXL );
                swap(_a.hsXH,_b.hsXH );
                swap(_a.hsYL,_b.hsYL );
                swap(_a.hsYH,_b.hsYH );
                swap(_a.stickX, _b.stickX);
                swap(_a.stickY, _b.stickY);
                swap(_a.UIvalue, _b.UIvalue);
                swap(_a.label, _b.label);
                swap(_a.msOverTxt, _b.msOverTxt);
                swap(_a.stsTxt, _b.stsTxt);
                swap(_a.clr	, _b.clr);
                swap(_a.onclick, _b.onclick);
                swap(_a.srcOfEvnt, _b.srcOfEvnt);
            }

            friend std::ostream& operator<<(std::ostream& out, const KinUIComponent& cmp);

        protected:		//functions
            virtual void init();																									//specifically designed to only be called by constructor

            //protected :
        protected :	//variables 
            static unsigned int ID_gen;
            int ID;
            int winX, winY;                                 //size of window holding object
            vector<bool> flags;							    //state flags for this object
            float x,y,w,h;								    //x,y location of upper left corner, width and height of actual control - slider button or push button
            float clkX, clkY;							    //last click x,y value
            float hsXL, hsXH, hsYL, hsYH, hsXBr, hsYBr;	    //hotspots - click area on screen, in screen coords - depends on UI, in mouse click coords
            float stickX, stickY;						    //location of sticking point in control for kinect-driven hands 
            vector<float> UIvalue;						    //value of control when activated, based on event type  TODD
            string label, msOverTxt, stsTxt;                //current label, mouse over popup text, status bar text(each should default to label if empty)
            vector<float> clr;							    //color of object, 0-1 for each value
            string onclick;                              
            int srcOfEvnt;                                  //source object of currently engaged event - KC_LEFT: left hand, KC_RIGHT : right hand, KC_MOUSE: mouse

        public:
            KinectController* kinectController;

        };//KinUIComponent
    }//namespace kinect
}//namespace rtql8
#endif
