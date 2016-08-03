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
#ifndef __MYKINCONTROLLER_H__
#define __MYKINCONTROLLER_H__

#include "kinect/KinectController.h"

using namespace rtql8::kinect;

namespace rtql8 {
    namespace kinematics {
        class Skeleton;
    }
}

namespace rtql8 {
    namespace optimizer {
        class ObjectiveBox;
        class Var;
    }
}

class MyKinController : public rtql8::kinect::KinectController {
public:
    MyKinController();
    ~MyKinController();
    //init/timer/draw methods - customize for each app in child class
    virtual void initWinVals(rtql8::renderer::RenderInterface* _mRI, int _wx, int _wy){
        KinectController::initWinVals(_mRI, _wx, _wy);
    }
    virtual bool parseSpeechCommand();
    virtual void setDispButtons();
    virtual bool checkUIElements(int& x, int& y, int& d, bool isMouse, int hand, int evnt);
    //build UI components
    virtual void build2dUI(vector<int>& numCmps);
    virtual void buildIKUI(vector<int>& numCmps);
    virtual int initIKSkel();															//any initialization the skeleton from the file may need after being loaded(modify location/orientation)
    virtual int initKinData();															//any initialization child class might require - called from onInit
    virtual void onTimerQuery();														//get info for ui display
    virtual void onDrawUpdate();														//display kinect results
    virtual void onDrawUpdate(rtql8::renderer::RenderInterface* mRI);					//display kinect results
    virtual void drawKinSkelData();														//draw various kinect-generated skeleton constructs

};//MyKinController class

#endif
