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

#include "ClosedLoopConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace rtql8::utils;

namespace rtql8 {
    namespace dynamics {
        ClosedLoopConstraint::ClosedLoopConstraint(SkeletonDynamics *_skel, BodyNodeDynamics *_body1, BodyNodeDynamics *_body2, Vector3d _offset1, Vector3d _offset2, bool _approx, double _timestep) {
            mSkel = _skel;
            mBody1 = _body1;
            mBody2 = _body2;
            mOffset1 = _offset1;
            mOffset2 = _offset2;
            mApproxJDot = _approx;
            mTimestep = _timestep;
            mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
            mPreJ = MatrixXd::Zero(3, mSkel->getNumDofs());
            mJDot = MatrixXd::Zero(3, mSkel->getNumDofs());
        }

        ClosedLoopConstraint::~ClosedLoopConstraint() {
        }
        
        void ClosedLoopConstraint::updateDynamics() {
            mPreJ = mJ;
            getJacobian();            
            getJacobianDot();
            Vector3d worldP1 = xformHom(mBody1->getWorldTransform(), mOffset1);
            Vector3d worldP2 = xformHom(mBody2->getWorldTransform(), mOffset2);
            VectorXd qDot = mSkel->getQDotVector();
            mC = worldP1 - worldP2;
            mCVel = mJ * qDot;
        }

        void ClosedLoopConstraint::getJacobian() {
            mJ.setZero();
            for(int i = 0; i < mBody1->getNumDependentDofs(); i++) {
                int dofIndex = mBody1->getDependentDof(i);
                VectorXd Jcol = xformHom(mBody1->getDerivWorldTransform(i), mOffset1);
                mJ.col(dofIndex) = Jcol;
            }
            for(int i = 0; i < mBody2->getNumDependentDofs(); i++) {
                int dofIndex = mBody2->getDependentDof(i);
                VectorXd Jcol = xformHom(mBody2->getDerivWorldTransform(i), mOffset2);
                mJ.col(dofIndex) -= Jcol;
            }
        }

        void ClosedLoopConstraint::getJacobianDot() {
            mJDot.setZero();
            if (mApproxJDot) {
                mJDot = (mJ - mPreJ) / mTimestep;
            } else {
                VectorXd qDot = mSkel->getQDotVector();
                int nLocalDof = mBody1->getNumDependentDofs();
                MatrixXd sum1(MatrixXd::Zero(3, nLocalDof));
                mBody1->updateSecondDerivatives(mOffset1);
                for (int i = 0; i < nLocalDof; i++) {
                    int dofIndex = mBody1->getDependentDof(i);
                    sum1 += mBody1->getJvDeriv(i) * qDot[dofIndex];
                }
                for (int i = 0; i < nLocalDof; i++) {
                    int dofIndex = mBody1->getDependentDof(i);
                    mJDot.col(dofIndex) = sum1.col(i);
                }

                nLocalDof = mBody2->getNumDependentDofs();
                MatrixXd sum2(MatrixXd::Zero(3, nLocalDof));
                mBody2->updateSecondDerivatives(mOffset2);
                for (int i = 0; i < nLocalDof; i++) {
                    int dofIndex = mBody2->getDependentDof(i);
                    sum2 += mBody2->getJvDeriv(i) * qDot[dofIndex];
                }

                for (int i = 0; i < nLocalDof; i++) {
                    int dofIndex = mBody2->getDependentDof(i);
                    mJDot.col(dofIndex) -= sum2.col(i);
                }
            }
        }
    } // namespace dynamics
} // namespace rtql8

