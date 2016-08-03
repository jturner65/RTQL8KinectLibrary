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
#include "MyKinController.h"

using namespace rtql8::kinect;

MyKinController::MyKinController() : KinectController() {

//   cout<<"read MyKinController xml file"<<endl;
//   rtql8::kinect::KinCntrlrXMLReader config(KC);
//   config.readFile(RTQL8_DATA_PATH"kinectUIConfigJT.xml");
//   cout<<"done reading MyKinController xml file"<<endl;

	setFlag(_KC_PROC_IK, true);
	StartKinIKSolver();
    setFlag(_KC_PROC_HANDS_2D, true);

}//cnstrctr

MyKinController::~MyKinController() {
}//dstrctr

///////
//initializations
///////

//any initialization the IK skeleton/avatar may need after being loaded (set initial position/orientation)
//return 0 for no error, 2 for error in initialization
int MyKinController::initIKSkel(){
    int numD = kinSkel->getNumDofs();
    Eigen::VectorXd q(numD);
    //modify skeleton's initial position to face camera and stand a bit more naturally
    kinSkel->getPose(q);
    q[4] = -2.1;															//legs - slightly askew
    q[9] = -0.17;
    q[15]= -0.17;
    cout<<"set skel pose in init"<<endl;
    kinSkel->setPose(q, true, true);
    return 0;
}//initIKSkel

//any initialization that any instance-specific data might need
//return 0 for no error, 2 for error in initialization
int MyKinController::initKinData(){
    return 0;
}//initIKSkel


//build demo UI components - instantiate via base class call, initialize values/customize here
void MyKinController::build2dUI(vector<int>& numCmps){
    KinectController::build2dUI(numCmps);																	//needs to be first! instantiates objects

    //DEMO STUFF : 
    string butLblNames[] = {"Move","Rotate", "Flex", "LGrip ", "RGrip ", "", ""};
    string sldLblNames[] = {"Angle", "Amount", "Speed","",""};									//labels for demo UI elements
    string crnkLblNames[] = {"Deg 1","Deg 2", "", ""};
    string imgButLblNames[] = {"imgBut 1","Anim 2", "", ""};
    string prgBarLblNames[] = {"Progress:", "", ""};
    string timerLblNames[] = {"CD:", "", ""};
    string tbLblNames[] = {"TextBox 1:", "TextBox 2:", ""};
    string capLblNames[] = {"Caption 1:", "Caption 2:", ""};

    string butMONames[] = {"Move Button","Rotate Button", "Flex Button", "Left Grip ", "Right Grip ","",""};
    string sldMONames[] = {"Angle Slider", "Amount Slider", "Speed Slider","",""};				//labels for demo UI elements
    string crnkMONames[] = {"Angle Crank 1","Angle Crank 2","",""};
    string imgButMONames[] = {"Image button 1","Animation 2","",""};
    string prgBarMONames[] = {"Progress Bar","",""};
    string timerMONames[] = {"Count Down","",""};
    string tbMONames[] = {"Text Box 1","Text Box 2",""};
    string capMONames[] = {"Caption Text 1","Caption Text 2",""};


    int cmpIdx = 0;
    //starting values for buttons - upper left location, width, height, spacer
    //click zone needs to be scaled by .75 of actual coords being painted.
    int bx = 0, by = 0, bw = 120, bh = 75, bs = 70, _hsBrdrX = 35, _hsBrdrY = 50;
    if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx]; ++i){			//buttons
            Kin2dUIBtn[i]->setDisplayHotSpot(true);
            Kin2dUIBtn[i]->setLoc(bx,by);
            Kin2dUIBtn[i]->setDim(bw,bh);
            Kin2dUIBtn[i]->setColor(.65f,.65f,.65f,1);
            if(i<numCmps[0]-2){	Kin2dUIBtn[i]->setColor((i == 0) || (i == 3)? 1 : 0, (i == 1) || (i == 3) || (i == 4)? 1 : 0, (i == 2) || (i == 4) ? 1 : 0 ,1);}
            Kin2dUIBtn[i]->setLabel(butLblNames[i]);
            Kin2dUIBtn[i]->setMSOverTxt(butMONames[i]);
            Kin2dUIBtn[i]->setHotSpot(_hsBrdrX, _hsBrdrY);										//use to delimit area around control for hand to snap to
            bx += (bw + bs);
        }
        cmpIdx++;

	//starting values for sliders :		
    //_srng : range of slider values
	//_sp : 0-1 position along slider -> 0 at top or left, 1 at bottom or right (based on orientation) mult by srng to get value of slider
	//# of subdivisions of slider for discrete values - <= 0 means "continuous"
	//dimensions of sliderbar track (where slider thumb moves)
	//orientation of slider - true is horizontal, false is vertical

	float _srng = 100, _sp = .3f;
	//starting values for sliders - upper left location, width, height, spacer
	bx = 150, by = bh + bs, bw = 75, bh = 85, bs = 70, _hsBrdrX = 35, _hsBrdrY = 35;		
	int _sdv = 0, _slen = 200, _sthick = 40;
	if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx];  ++i){
            Kin2dUISldr[i]->setLoc(bx,by);
            Kin2dUISldr[i]->setDim(bw,bh);
            Kin2dUISldr[i]->setColor((i == 0) ? 0 : 1 ,(i == 1) ? 0 : 1, (i == 2) ? 0 : 1 ,1);
            Kin2dUISldr[i]->setLabel(sldLblNames[i]);
            Kin2dUISldr[i]->setMSOverTxt(sldMONames[i]);
            Kin2dUISldr[i]->setSlideVals(_srng,_sp * (i+1),_sdv,_slen,_sthick);
            Kin2dUISldr[i]->setIsHoriz((i != numCmps[1]-1 ? true : false));				//make last one vertical
            Kin2dUISldr[i]->setIsLToR((i != numCmps[1]-1 ? true : false));				//make last one increasing up
            Kin2dUISldr[i]->setHotSpot(_hsBrdrX, _hsBrdrY);								//use to delimit area around control for hand to snap to
            by += (bh + bs);
        }
        cmpIdx++;

    _srng = 360; _sp = .3f;
    //starting values for cranks - upper left location, width, height, spacer
    bx = 600, by = 400, bw = 100, bh = 70, bs = 80, _hsBrdrX = 80, _hsBrdrY = 80;		
    int _sdv = 0, _slen = 250, _sthick = 20;	
    if(numCmps.size() > cmpIdx){
		for(int i = 0; i < numCmps[cmpIdx];  ++i){
            Kin2dUICrnk[i]->setLoc(bx,by);
            Kin2dUICrnk[i]->setDim(bw,bh);
            Kin2dUICrnk[i]->setColor((i == 0) ? 0 : 1 ,(i == 1) ? 0 : 1, (i == 2) ? 0 : 1 ,1);
            Kin2dUICrnk[i]->setLabel(crnkLblNames[i]);
            Kin2dUICrnk[i]->setMSOverTxt(crnkMONames[i]);
            Kin2dUICrnk[i]->setCrankVals(0, _srng,_sp * (i+1),_sdv,_slen,_sthick);
            Kin2dUICrnk[i]->setIsClockWise((i != numCmps[1]-1 ? true : false));				
            Kin2dUICrnk[i]->setHotSpot(_hsBrdrX, _hsBrdrY);							//use to delimit area around control for hand to snap to - set x and y borders
            by += (bh + bs);
		}
        cmpIdx++;

    //starting values for image buttons - upper left location, width, height, spacer
    bx = 600, by = 200, bw = 200, bh = 150, bs = 80, _hsBrdrX = 35, _hsBrdrY = 50;		
    int _sdv = 0, _slen = 250, _sthick = 20;	
    std::string imgname = "smile.jpg";
    if(numCmps.size() > cmpIdx){
		for(int i = 0; i < numCmps[cmpIdx];  ++i){
            Kin2dUIImgBtn[i]->setLoc(bx,by);
            Kin2dUIImgBtn[i]->setDim(bw,bh);
            Kin2dUIImgBtn[i]->setColor((i == 0) ? 0 : 1 ,(i == 1) ? 0 : 1, (i == 2) ? 0 : 1 ,1);
            Kin2dUIImgBtn[i]->setLabel(imgButLblNames[i]);
            Kin2dUIImgBtn[i]->setMSOverTxt(imgButMONames[i]);
            Kin2dUIImgBtn[i]->setHotSpot(_hsBrdrX, _hsBrdrY);							//use to delimit area around control for hand to snap to - set x and y borders
            Kin2dUIImgBtn[i]->setImgSrcFileName(RTQL8_DATA_PATH + imgname);
            by += (bh + bs);
		}
        cmpIdx++;

	//starting values for progress bars - upper left location, width, height, spacer
	bx = 850, by = 500, bw = 50, bh = 50, bs = 25, _hsBrdrX = 0, _hsBrdrY = 0;		
	int _sdv = 0, _slen = 250, _sthick = 10;
	if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx];  ++i){
            Kin2dUIProgBar[i]->setLoc(bx,by);
            Kin2dUIProgBar[i]->setDim(bw,bh);
            Kin2dUIProgBar[i]->setColor(0, 0, 1 ,1);
            Kin2dUIProgBar[i]->setLabel(prgBarLblNames[i]);
            Kin2dUIProgBar[i]->setMSOverTxt(prgBarMONames[i]);
            Kin2dUIProgBar[i]->setSlideVals(100,0,_sdv,_slen,_sthick);
            Kin2dUIProgBar[i]->setIsHoriz( true );			
            Kin2dUIProgBar[i]->setHotSpot(_hsBrdrX, _hsBrdrY);								//use to delimit area around control for hand to snap to
            Kin2dUIProgBar[i]->setIsLToR( true);                                            //direction : this prog bar is left to right
            Kin2dUIProgBar[i]->reset();
            by += (bh + bs);
        }
        cmpIdx++;        
	//starting values for timers - upper left location, width, height, spacer
	bx = 850, by = 650, bw = 50, bh = 50, bs = 25, _hsBrdrX = 0, _hsBrdrY = 0;		
	int _sdv = 0, _slen = 250, _sthick = 10;
	if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx];  ++i){
            Kin2dUITimer[i]->setLoc(bx,by);
            Kin2dUITimer[i]->setDim(bw,bh);
            Kin2dUITimer[i]->setColor(0 == i ?1 : .5f, 0 ,0 == i ? 0.5f : 0 ,1);
            Kin2dUITimer[i]->setLabel(timerLblNames[i]);
            Kin2dUITimer[i]->setMSOverTxt(timerMONames[i]);
            Kin2dUITimer[i]->setSlideVals(5,0,_sdv,_slen,_sthick);
            Kin2dUITimer[i]->setIsHoriz( true );			
            Kin2dUITimer[i]->setHotSpot(_hsBrdrX, _hsBrdrY);								//use to delimit area around control for hand to snap to
            Kin2dUITimer[i]->setIsCntDn(0 == i ? true : false);                             //idx0 is stopwatch, others are countdown
            Kin2dUITimer[i]->setIsLToR(0 == i ? false : true);                              //direction - this timer moves right to left if false, left to right if true
            Kin2dUITimer[i]->reset();
            by += (bh + bs);
        }
        cmpIdx++;   
	//starting values for textbox - upper left location, width, height, spacer
	bx = 850, by = 150, bw = 120, bh = 50, bs = 50, _hsBrdrX = 35, _hsBrdrY = 50;		
	int _sdv = 0, _slen = 250, _sthick = 10;
	if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx];  ++i){
            Kin2dUITextBox[i]->setLoc(bx,by);
            Kin2dUITextBox[i]->setDim(bw,bh);
            Kin2dUITextBox[i]->setMaxChars((i+1) * 5);
            Kin2dUITextBox[i]->setColor((i == 0) ? 0 : 1 ,(i == 1) ? 0 : 1, (i == 2) ? 0 : 1 ,1);
            Kin2dUITextBox[i]->setLabel(tbLblNames[i]);
            Kin2dUITextBox[i]->setMSOverTxt(tbMONames[i]);
            Kin2dUITextBox[i]->setHotSpot(_hsBrdrX, _hsBrdrY);								//use to delimit area around control for hand to snap to
            by += (bh + _hsBrdrY + bs);
        }
        cmpIdx++;       
	bx = 500, by = 700, bw = 120, bh = 50, bs = 50, _hsBrdrX = 35, _hsBrdrY = 50;		
	int _sdv = 0, _slen = 250, _sthick = 10;
	if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx];  ++i){
            Kin2dUICaption[i]->setLoc(bx,by);
            Kin2dUICaption[i]->setDim(bw,bh);
            Kin2dUICaption[i]->setColor((i == 0) ? 0 : 1 ,(i == 1) ? 0 : 1, (i == 2) ? 0 : 1 ,1);
            Kin2dUICaption[i]->setLabel(capLblNames[i]);
            Kin2dUICaption[i]->setMSOverTxt(capMONames[i]);
            Kin2dUICaption[i]->setHotSpot(0, 0);								//use to delimit area around control for hand to snap to
            by += (bh + _hsBrdrY + bs);
        }
        cmpIdx++;        

        //insert img buttons and timers here if any
    }}}}}}}}//checking size of vector holding number of components
    clog<<"Num buttons : "<<Kin2dUIBtn.size()<<endl;
    clog<<"Num sliders : "<<Kin2dUISldr.size()<<endl;
    clog<<"Num cranks : "<<Kin2dUICrnk.size()<<endl;
    clog<<"Num imgButtons : "<<Kin2dUIImgBtn.size()<<endl;
    clog<<"Num progress bars : "<<Kin2dUIProgBar.size()<<endl;
    clog<<"Num timers : "<<Kin2dUITimer.size()<<endl;
    clog<<"Num tbs : "<<Kin2dUITextBox.size()<<endl;
    clog<<"Num captions : "<<Kin2dUICaption.size()<<endl;
    clog<<"Num total : "<<Kin2dUICmp.size()<<endl;
    for(int i = 0; i<Kin2dUICmp.size(); ++i){
        Kin2dUICmp[i]->setDisplayHotSpot(true);
        Kin2dUICmp[i]->setWinSize(winX, winY);
        cout<<*Kin2dUICmp[i]<<endl;
    }
}//build2dUI

void MyKinController::buildIKUI(vector<int>& numCmps){
    KinectController::buildIKUI(numCmps);

    //DEMO STUFF : 
    string butLblNames[] = {"IK but1","", ""};
    string prgBarlblNames[] = {"Progress:","",""};
    string timerlblNames[] = {"Count Down : ","Timer :","",""};

    string butMONames[] = {"IK Button","",""};
    string prgBarMONames[] = {"IK Progress bar","",""};
    string timerMONames[] = {"IK Timer","",""};
    int cmpIdx = 0;
    //starting values for buttons - upper left location, width, height, spacer
    //click zone needs to be scaled by .75 of actual coords being painted.
    int bx = 0, by = 0, bw = 120, bh = 75, bs = 70, _hsBrdrX = 35, _hsBrdrY = 50;
    if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx]; ++i){			//buttons
            KinIKUIBtn[i]->setLoc(bx,by);
            KinIKUIBtn[i]->setDim(bw,bh);
            KinIKUIBtn[i]->setColor(.65f,.65f,.65f,1);
            if(i<numCmps[0]-2){	KinIKUIBtn[i]->setColor((i == 0) || (i == 3)? 1 : 0, (i == 1) || (i == 3) || (i == 4)? 1 : 0, (i == 2) || (i == 4) ? 1 : 0 ,1);}
            KinIKUIBtn[i]->setLabel(butLblNames[i]);
            KinIKUIBtn[i]->setMSOverTxt(butMONames[i]);
            KinIKUIBtn[i]->setHotSpot(_hsBrdrX, _hsBrdrY);										//use to delimit area around control for hand to snap to
            bx += (bw + bs);
        }//for
        cmpIdx++;

    float _srng = 100, _sp = .3f;
	//starting values for progressbars - upper left location, width, height, spacer
	bx = 250, by = 200, bw = 75, bh = 85, bs = 70, _hsBrdrX = 35, _hsBrdrY = 35;		
	int _sdv = 0, _slen = 200, _sthick = 40;
	if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx];  ++i){
            KinIKUIProgBar[i]->setLoc(bx,by);
            KinIKUIProgBar[i]->setDim(bw,bh);
            KinIKUIProgBar[i]->setColor((i == 0) ? 0 : 1 ,(i == 1) ? 0 : 1, (i == 2) ? 0 : 1 ,1);
            KinIKUIProgBar[i]->setLabel(timerlblNames[i]);
            KinIKUIProgBar[i]->setMSOverTxt(timerMONames[i]);
            KinIKUIProgBar[i]->setSlideVals( 100 ,0,_sdv,_slen,_sthick);
            KinIKUIProgBar[i]->setIsHoriz( true );			
            KinIKUIProgBar[i]->setHotSpot(0, 0);			                                    //no hand zone - no interaction with timer and progress bars					    
            KinIKUIProgBar[i]->setIsLToR((i == 0 ? false : true));                            //direction - prog bar is left to right, timer is right to left
            KinIKUIProgBar[i]->reset();
            by += (bh + bs);
        }
        cmpIdx++; 

    float _srng = 100, _sp = .3f;
	//starting values for timers - upper left location, width, height, spacer
	bx = 850, by = 400, bw = 75, bh = 85, bs = 70, _hsBrdrX = 35, _hsBrdrY = 35;		
	int _sdv = 0, _slen = 200, _sthick = 40;
	if(numCmps.size() > cmpIdx){
        for(int i = 0; i < numCmps[cmpIdx];  ++i){
            KinIKUITimer[i]->setLoc(bx,by);
            KinIKUITimer[i]->setDim(bw,bh);
            KinIKUITimer[i]->setColor((i == 0) ? 0 : 1 ,(i == 1) ? 0 : 1, (i == 2) ? 0 : 1 ,1);
            KinIKUITimer[i]->setLabel(timerlblNames[i]);
            KinIKUITimer[i]->setMSOverTxt(timerMONames[i]);
            KinIKUITimer[i]->setSlideVals(5,0,_sdv,_slen,_sthick);
            KinIKUITimer[i]->setIsHoriz( true );			
            KinIKUITimer[i]->setHotSpot(0, 0);			                                    //no hand zone - no interaction with timer and progress bars					    
            KinIKUITimer[i]->setIsLToR((i == 0 ? false : true));                            //direction - prog bar is left to right, timer is right to left
            KinIKUITimer[i]->reset();
            by += (bh + bs);
        }
        cmpIdx++; 
    }}}
     
    clog<<"Num IK buttons : "<<KinIKUIBtn.size()<<endl;
    clog<<"Num IK progress bars : "<<KinIKUIProgBar.size()<<endl;
    clog<<"Num IK timers : "<<KinIKUITimer.size()<<endl;
    clog<<"Num total : "<<KinIKUICmp.size()<<endl;
    for(int i = 0; i<KinIKUICmp.size(); ++i){
        KinIKUICmp[i]->setDisplayHotSpot(false);
        KinIKUICmp[i]->setWinSize(winX, winY);
        cout<<"idx : "<<i<<" obj : "<<*KinIKUICmp[i]<<endl;
    }

}//buildIKUI

//will set the display of the grip/click info buttons so that they reflect what each hand is doing - 0 is nothing, 1 is click, 2 is grab
//these buttons are currently idx 3 and 4 in Kin2dUIBtn vector
void MyKinController::setDispButtons(){
     string lHandButAra[3] = {"LHand", "LClick", "LGrip "};
     string rHandButAra[3] = {"RHand", "RClick", "RGrip "};
     Kin2dUIBtn[3]->setLabel(lHandButAra[this->KinHands[KC_LEFT].getHandState()]);
     Kin2dUIBtn[4]->setLabel(rHandButAra[this->KinHands[KC_RIGHT].getHandState()]);
     //stringstream lbl;
     //lbl<<(this->KinHands[KC_LEFT].getFlag(_KHnd_HAND_IN_OBJ) ? "! -" : "o -")<<pushProgress[KC_LEFT]<<"- L:("<<(int)gLHandLoc(0)<<", "<<(int)gLHandLoc(1)<<", "<<(int)gLHandLoc(2)<<", "<<lHandSt<<")";			//in object show !, otherwise show :
     //Kin2dUICaption[0]->setLabel(lbl.str());																				//idx 5 for left button, 6 for right button - set label for button describing hand position
     //Kin2dUICaption[0]->setColor(KC_flags[_KC_CHNG_HANDS_2D]?1:0, KC_flags[_KC_CHNG_HANDS_2D]?1:0, KC_flags[_KC_CHNG_HANDS_2D]?0:1,1);			//if hands have changed, should turn this button yellow, otherwise should turn it blue
     //lbl.str("");
     //lbl.clear();
     //lbl<<(this->KinHands[KC_RIGHT].getFlag(_KHnd_HAND_IN_OBJ) ? "! -" : "o -")<<pushProgress[KC_RIGHT]<<"- R:("<<(int)gRHandLoc(0)<<", "<<(int)gRHandLoc(1)<<", "<<(int)gRHandLoc(2)<<", "<<rHandSt<<")";
     //Kin2dUICaption[1]->setLabel(lbl.str());																				                    //idx 5 for left button, 6 for right button - set label for button describing hand position
     //Kin2dUICaption[1]->setColor(KC_flags[_KC_MOUSE_USE]?0:1, KC_flags[_KC_MOUSE_USE]?1:0, KC_flags[_KC_MOUSE_USE]?1:0,1);					//if mouse event should turn this button cyan, otherwise should turn it red
}//setDispButtons

///////
//call these from timer loop
//////
//debug info for gui display
void MyKinController::onTimerQuery(){
    if(kinHNDLR->kinSkelHandlerAvail()){
        skelJointStrs =  kinHNDLR->getBestKinSkelJointLocsStrs();
        if(kinSpeechRecog != ""){this->parseSpeechCommand();}
    }//if valid skel handler
}//onTimerQuery

///////
///call these from draw
///////

void MyKinController::onDrawUpdate(rtql8::renderer::RenderInterface* mRI){
    glPushMatrix();
    glTranslatef(0,0,-3);
    if(nullptr != kinSkel){
        //cout<<"skel ! null"<<endl;
        glPolygonMode(GL_FRONT_AND_BACK,  GL_LINE);									//line around polys, only for skeleton
        if(KC_flags[_KC_DISP_AVATAR]){kinSkel->draw(mRI);}
        if(KC_flags[_KC_DISP_MRKRS]){kinSkel->drawMarkers(mRI);}
    }
    skelDrawn = true;
    glPopMatrix();
    onDrawUpdate();
}//onDrawUpdate

//display IK skeleton and various kinect-derived information and images
void MyKinController::onDrawUpdate(){
    onTimerQuery();																	//update ui info
    //kinect stuff drawn here 
    if(kinHNDLR->kinValid()) {
        kinectRConnTimer = 90;
        glPushMatrix();
        glTranslatef(0,-1,-3);
        if(KC_flags[_KC_DISP_DEPTH]){												//if we want depth image display
            drawKinDpthHandImage(true, -4,4,-5,1,1,1);								//left hand depth segmentation
            drawKinDpthHandImage(false, 4,4,-5,1,1,1);								//right hand depth segmentation
            drawKinImage(true,0,0,-5,1,1,1);										//draw depth image
        }//if using depth image
        if(KC_flags[_KC_DISP_RGB]){	drawKinImage(false,0,5,-5,.2f,.2f,1);	}		//if we want RGB image display draw rgb image
			
        if(KC_flags[_KC_USE_STRM_SKEL]){											//if we want IK on kinect skeleton
            glPushMatrix();
            glTranslatef(0,1,0);
            if(kinHNDLR->kinSkelHandlerAvail()){	drawKinSkelData();			}							//base-class available draw function - processing skeleton happens 
            glPopMatrix();
        }//if using skel stream/IK
        glPopMatrix();
    }//if kinect connected
    else if (0 >= kinectRConnTimer) {kinHNDLR->checkKinectConn();kinectRConnTimer = 90;}//attempt to reconnect to kinect if not valid/present
    else{	--kinectRConnTimer;}

    //draw skeleton visualisations
    if((nullptr != kinSkel) && (!skelDrawn)){
        if(KC_flags[_KC_DISP_AVATAR]){kinSkel->draw();}
        if(KC_flags[_KC_DISP_MRKRS]){kinSkel->drawMarkers();}
    }
    KinectController::onDrawUpdate();											//needs to be here
    skelDrawn = false;

}//onDrawUpdate

bool MyKinController::parseSpeechCommand(){
    bool res = KinectController::parseSpeechCommand();
    //moved to base class
    //if (kinSpeechRecog == "IK MODE"){
    //    clog<<"MyKinController : Change to IK mode"<<endl;
    //    setFlag(_KC_PROC_IK, true);
    //    clearSpeechRecogVals();
    //} else if (kinSpeechRecog == "UI MODE"){
    //    clog<<"MyKinController : Change to UI mode"<<endl;
    //    setFlag(_KC_PROC_HANDS_2D, true);
    //    clearSpeechRecogVals();
    //} else if (kinSpeechRecog == "3D MODE"){
    //    clog<<"MyKinController : Change to 3D mode"<<endl;
    //    setFlag(_KC_PROC_HANDS_3D, true);
    //    clearSpeechRecogVals();
    //} else {
    if(res){    
        return res;
    } else {//not found in base class
        //add custom phrase handling in here
        clog<<"MyKinController : Unhandled Recognized Phrase : "<<kinSpeechRecog<<endl;
        clearSpeechRecogVals();
    }
    return res;
	
}//parseSpeechCommand

//draw these programmatically
//custom handling - basic handling done in base class
bool MyKinController::checkUIElements(int& x, int& y, int& d, bool isMouse, int hand, int evnt){
    bool clickedOn = false;
    //call base class version to handle actual object events
    bool result = KinectController::checkUIElements(x, y, d, isMouse, hand, evnt);

    return result;
}//checkUIElements

//draw various kinect and avatar skeleton and skeleton-related structures
void MyKinController::drawKinSkelData(){
    if(KC_flags[_KC_DISP_SKCLIP]){										drawKinSkelClipBox(kinSkelDisp[SK_NSY_KIN][0],kinSkelDisp[SK_NSY_KIN][1],kinSkelDisp[SK_NSY_KIN][2]);}
    if((KC_flags[_KC_DISP_RAWSK]) || (KC_flags[_KC_DISP_FILTSK])){		kinSkelJointState = kinHNDLR->getCtlSkelJointState(); }		//if displaying either filtered or raw skeleton data, then get joint state info
    if(KC_flags[_KC_DISP_AVTLOC]){										drawKinSkelFrame(kinSkelDisp[SK_AVTR][0],kinSkelDisp[SK_AVTR][1],kinSkelDisp[SK_AVTR][2],SK_AVTR, avatarIKHandles);	}
    if((KC_flags[_KC_DISP_RAWSK]) && (kinHNDLR->validCtlSkel())){		drawKinSkelFrame(kinSkelDisp[SK_NSY_KIN][0],kinSkelDisp[SK_NSY_KIN][1],kinSkelDisp[SK_NSY_KIN][2],SK_NSY_KIN, kinSkelNoisyMarkers);}
    if((KC_flags[_KC_DISP_FILTSK]) && (kinHNDLR->validCtlSkel())){		drawKinSkelFrame(kinSkelDisp[SK_FLT_KIN][0],kinSkelDisp[SK_FLT_KIN][1],kinSkelDisp[SK_FLT_KIN][2], SK_FLT_KIN, kinSkelMarkerPos);}
}//drawKinSkelData
