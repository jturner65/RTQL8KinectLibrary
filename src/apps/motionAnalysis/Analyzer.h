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


#ifndef ANALYZER_H
#define ANALYZER_H

#include <Eigen/Dense>
#include <vector>

namespace rtql8 {
    namespace dynamics {
        class SkeletonDynamics;
    }
}

namespace rtql8 {
    namespace kinematics {
        class FileInfoDof;
    }
}

class Analyzer {
 public:
    Analyzer(rtql8::kinematics::FileInfoDof *_motion, rtql8::dynamics::SkeletonDynamics *_skel, Eigen::Vector3d _grav);
    virtual ~Analyzer() {};

    void analyze();
    rtql8::dynamics::SkeletonDynamics* getSkel() { return mSkel; };

 protected: 
    void computeTorques();
    Eigen::Vector3d evalLinMomentum(const Eigen::VectorXd& _dofVel);
    Eigen::Vector3d evalAngMomentum(const Eigen::VectorXd& _dofVel);

    rtql8::dynamics::SkeletonDynamics *mSkel;
    rtql8::kinematics::FileInfoDof *mMotion;
    std::vector<Eigen::VectorXd> mTorques;
    std::vector<Eigen::Vector3d> mLinMomentum;
    std::vector<Eigen::Vector3d> mAngMomentum;
    
    Eigen::Vector3d mGravity;
};
    
    

#endif // #ANALYZER_H
