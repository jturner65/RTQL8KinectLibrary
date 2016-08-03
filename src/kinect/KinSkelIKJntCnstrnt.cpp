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

#include "KinHndlrConsts.h"
#include "KinSkelIKJntCnstrnt.h"

using namespace rtql8::kinematics;
using namespace rtql8::utils;
using namespace rtql8::optimizer;

namespace rtql8 {
    namespace kinect {

        KinSkelIKJntCnstrnt::KinSkelIKJntCnstrnt(vector<Var *>& var, Skeleton* skel, BodyNode* node, const Eigen::Vector3d& offset, const Eigen::Vector3d& val) : 
				Constraint(var), mSkel(skel), mNode(node), mTarget(val), mOffset(offset), name("") {
            mNumRows = 3;
            mWeight = Eigen::VectorXd::Ones(mNumRows);
            mConstTerm = Eigen::VectorXd::Zero(mNumRows);
            mCompletion = Eigen::VectorXd::Zero(mNumRows);
        }

        Eigen::VectorXd KinSkelIKJntCnstrnt::evalCon() {
            Eigen::Vector3d wp = mNode->evalWorldPos(mOffset);
            Eigen::Vector3d c = wp - mTarget;
            Eigen::VectorXd ret(c);
            return ret;
        }

        void KinSkelIKJntCnstrnt::fillJac(VVD jEntry, VVB jMap, int index) {
            for(int i = 0; i < mNode->getNumDependentDofs(); i++) {
                int dofindex = mNode->getDependentDof(i);
                const Var* v = mVariables[dofindex];
                Eigen::VectorXd J = xformHom(mNode->getDerivWorldTransform(i), mOffset);
                for (int k = 0; k < 3; k++) {
                    (*jEntry)[index + k]->at(dofindex) = J[k];
                    (*jMap)[index + k]->at(dofindex) = true;
                }
            }
        }

        void KinSkelIKJntCnstrnt::fillObjGrad(std::vector<double>& dG) {
           Eigen::VectorXd dP = evalCon();
            for(int i = 0; i < mNode->getNumDependentDofs(); i++) {
                int dofindex = mNode->getDependentDof(i);
                Eigen::VectorXd J = xformHom(mNode->getDerivWorldTransform(i), mOffset);
                dG[dofindex] += 2 * dP.dot(J);
            }
        }

        void KinSkelIKJntCnstrnt::setTarget(const Eigen::Vector3d& target) {        mTarget = target;      }
        Eigen::Vector3d KinSkelIKJntCnstrnt::getTarget() const {				    return mTarget;        }
    } // namespace optimizer
} // namespace rtql8
