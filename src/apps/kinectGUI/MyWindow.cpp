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

#include "MyWindow.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include "kinect/KinCntrlrXMLReader.h"

using namespace rtql8::utils;
using namespace rtql8::optimizer;
using namespace rtql8::kinematics;

extern MyKinController* KC;


void MyWindow::initWindow(int w, int h, const char* name){
	GlutWindow::initWindow(w,h,name);				//base class
	glutTimerFunc ( mDisplayTimeout, refreshTimer, (1000.0/30.0) );

    vector<int> tmpVec = vector<int>();
    tmpVec.push_back(7); //number of buttons
    tmpVec.push_back(3); //number of sliders
    tmpVec.push_back(1); //number of cranks
    //tmpVec.push_back(1); //number of img buttons
    tmpVec.push_back(0); //number of img buttons
    tmpVec.push_back(1); //number of prog bars
    tmpVec.push_back(2); //number of timers
    tmpVec.push_back(2); //number of text boxes
    tmpVec.push_back(2); //number of captions
    KC->build2dUI(tmpVec);	

    vector<int> tmpVec1 = vector<int>();
    tmpVec1.push_back(0); //number of IK disp only buttons
    tmpVec1.push_back(1); //number of IK disp only prog bars
    tmpVec1.push_back(1); //number of IK disp only timers
    KC->buildIKUI(tmpVec1);

	//build ui with kinect-interacting objects - to be replaced with XML
    //clog<<"kinectGUI::MyWindow : read  xml file"<<endl;
    //rtql8::kinect::KinCntrlrXMLReader config(KC);
    //config.readFile(RTQL8_DATA_PATH"kinectUIConfigJT.xml");
    //clog<<"kinectGUI::MyWindow : done reading xml file"<<endl;

	KC->initWinVals(mRI, w, h);

}//initWindow

void MyWindow::displayTimer(int _val){
	//cout<<"disp timer val : "<<_val<<endl;
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::onTimerIdle() {	
	KC->onTimerUpdate();
}


void MyWindow::click(int button, int state, int x, int y){
	//button : left : 0, middle : 1, right : 2
	//state : down : 0, up : 1
	//x,y in screen coords
	//does not handle menu bar, does not handle drag
	//state is 0 for click, 1 for click up - use 0 for up, 1 for click
    int depth = 0;
	if (!KC->checkUIElements(x, y, depth, true, button, 1 - state))	{			//call parent first, parent calls child
        //if no element clicked on, call parent window to handle view control/trackball
		Win3D::click(button, state, x, y);
	} else {
        this->mMouseX = x;
        this->mMouseY = y;
    }
}

void MyWindow::drag(int x, int y){
    int depth = 0;
	if (!KC->checkUIElements(x, y, depth, true, 0, 2))	{						//drag event with left click
		Win3D::drag(x,y);
	} else {
        this->mMouseX = x;
        this->mMouseY = y;
    }
}

void MyWindow::draw(){
    glDisable(GL_LIGHTING);
	KC->onDrawUpdate(mRI);
	glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y){
    if(!KC->onKeyUpdate(key, x, y)) {
        Win3D::keyboard(key,x,y);
	    // switch(key){
            // default:         Win3D::keyboard(key,x,y);
            // }
    }
    glutPostRedisplay();
}//keyboard
