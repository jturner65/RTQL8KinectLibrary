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

#include "PointConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace rtql8::utils;

namespace rtql8 {
    namespace dynamics {
        PointConstraint::PointConstraint(SkeletonDynamics *_skel, BodyNodeDynamics *_body, Vector3d _offset, Vector3d _target, bool _approx, double _timestep) {
            mSkel = _skel;
            mBody = _body;
            mOffset = _offset;
            mTarget = _target;
            mApproxJDot = _approx;
            mTimestep = _timestep;
            mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
            mPreJ = MatrixXd::Zero(3, mSkel->getNumDofs());
            mJDot = MatrixXd::Zero(3, mSkel->getNumDofs());
        }

        PointConstraint::~PointConstraint() {
        }
        
        void PointConstraint::updateDynamics() {
            mPreJ = mJ;
            getJacobian();
            getJacobianDot();
            Vector3d worldP = xformHom(mBody->getWorldTransform(), mOffset);
            VectorXd qDot = mSkel->getQDotVector();
            mC = worldP - mTarget;
            mCVel = mJ * qDot;
        }

        void PointConstraint::getJacobian() {
            for(int i = 0; i < mBody->getNumDependentDofs(); i++) {
                int dofIndex = mBody->getDependentDof(i);
                VectorXd Jcol = xformHom(mBody->getDerivWorldTransform(i), mOffset);
                mJ.col(dofIndex) = Jcol;
            }
        }

        void PointConstraint::getJacobianDot() {
            if (mApproxJDot) {
                mJDot = (mJ - mPreJ) / mTimestep;
            } else {
                int nLocalDof = mBody->getNumDependentDofs();
                VectorXd qDot = mSkel->getQDotVector();
                MatrixXd sum(MatrixXd::Zero(3, nLocalDof));
                mBody->updateSecondDerivatives(mOffset);
                for (int i = 0; i < nLocalDof; i++) {
                    int dofIndex = mBody->getDependentDof(i);
                    sum += mBody->getJvDeriv(i) * qDot[dofIndex];
                }
                for (int i = 0; i < nLocalDof; i++) {
                    int dofIndex = mBody->getDependentDof(i);
                    mJDot.col(dofIndex) = sum.col(i);
                }
            }
        }

        int PointConstraint::bodyIndex() {
            return mBody->getSkelIndex();
        }


    } // namespace dynamics
} // namespace rtql8

