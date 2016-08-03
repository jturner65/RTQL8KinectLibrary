/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
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

#include "Controller.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoDof.h"

using namespace Eigen;

Controller::Controller(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, Vector3d _grav) {
    mMotion = _motion;
    mSkel = _skel;
    mGravity = _grav;
    int nDof = mSkel->getNumDofs();
    int nFrame = mMotion->getNumFrames();
    mTorques.resize(nFrame);
    for (int i = 0; i < nFrame; i++) {
        mTorques[i].resize(nDof);
        mTorques[i].setZero();
    }
}

void Controller::computeTorques() {
    int nFrame = mMotion->getNumFrames();
    double timestep = 1.0 / mMotion->getFPS();
    for (int i = 0; i < nFrame - 2; i++) {
        VectorXd qdot = (mMotion->getPoseAtFrame(i + 1) - mMotion->getPoseAtFrame(i)) / timestep;

        VectorXd qddot = (mMotion->getPoseAtFrame(i + 2) - 2 * mMotion->getPoseAtFrame(i + 1)+ mMotion->getPoseAtFrame(i)) / (timestep * timestep);
        mSkel->setPose(mMotion->getPoseAtFrame(i), true, false);
        mTorques[i] = mSkel->computeInverseDynamicsLinear(mGravity, &qdot, &qddot, false, false);
    }
}

Eigen::VectorXd& Controller::getTorques(int _frame) {
    if (_frame < mTorques.size())
        return mTorques[_frame];
    else
        return mTorques[0];
}
