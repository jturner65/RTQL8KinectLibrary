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

#ifndef __KINLIBHELPER_H__
#define __KINLIBHELPER_H__

#include <Windows.h>
#include <random>
#include <iostream>
#include <memory>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <map>				//audio uses - to handle grammar
#include <stack>			//audio uses
#include <queue>			//audio uses
#include <deque>
#include <array>
#include <stdlib.h>
#include <stdio.h>
#include <propsys.h>		//audio

// TiCPP library
// http://code.google.com/p/ticpp/
#include "ticpp.h"

//RTQL8-specific files
#include "utils/UtilsMath.h"
#include "utils/Paths.h"

#include "optimizer/Constraint.h"
#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"

#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/Skeleton.h"
#include "kinematics/Dof.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Marker.h"

#include "yui/GLFuncs.h"

//eigen - needs to be defined after rtql8 functions
#define  EIGEN_DONT_ALIGN_STATICALLY		
#include <Eigen/Dense>

//GL libraries
#include <utils/LoadOpengl.h>
#include "yui/GLFuncs.h"

//SOIL image library functions (load images from files directly into textures)
#include <SOIL.h>

//kinect specific api's
#include <NuiApi.h>
#include <NuiSensor.h>
#include <NuiImageCamera.h>
#include <NuiSkeleton.h>
#include <KinectInteraction.h>	

//audio-related headers
#include <dmo.h>										// media-related 
#include <mmreg.h>										// For WAVEFORMATEX
#include <avrt.h>										// For MMCSS functionality such as AvSetMmThreadCharacteristics
#include <wmcodecdsp.h>									// For configuring DMO properties
#include <uuids.h>										// For FORMAT_WaveFormatEx and such

// For speech APIs
// NOTE: To ensure that application compiles and links against correct SAPI versions (from Microsoft Speech
//       SDK), VC++ include and library paths should be configured to list appropriate paths within Microsoft
//       Speech SDK installation directory before listing the default system include and library directories,
//       which might contain a version of SAPI that is not appropriate for use together with Kinect sensor.
//
//		tl;dr : need to have the correct version of speech api's to work with kinect - use those that are bundled
//		with kinect sdk (newest speech apis might not work with kinect) not those that come with windows or the speech sdk

#include <sapi.h>
#include <sphelper.h>



#endif//__KINLIBHELPER_H__