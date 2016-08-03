/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>, Yuting Ye <yuting.ye@gmail.com>
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

#include "simulation/World.h"

//using namespace Eigen;

//using namespace rtql8;
//using namespace kinematics;
//using namespace dynamics;
//using namespace simulation;

namespace rtql8 {
    namespace simulation {
        World::World() {

        }

        World::~World() {

        }

        void World::init() {
            int sumNDofs = 0;
            mIndices.push_back(sumNDofs);
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int nDofs = mSkels[i]->getNumDofs();
                sumNDofs += nDofs;
                mIndices.push_back(sumNDofs);
            }

            mDofs.resize(mSkels.size());
            mDofVels.resize(mSkels.size());

            for (unsigned int i = 0; i < mSkels.size(); i++) {
                mDofs[i].resize(mSkels[i]->getNumDofs());
                mDofVels[i].resize(mSkels[i]->getNumDofs());
                mDofs[i].setZero();
                mDofVels[i].setZero();
            }

            for (std::vector<dynamics::SkeletonDynamics*>::iterator itrSkels = mSkels.begin();
                 itrSkels != mSkels.end(); itrSkels++)
            {
                (*itrSkels)->initDynamics();
                //(*itrSkels)->setPose(mDofs, false, false); // set flags to skip transformation and first-derivatives updates
            }

            // set the ground to be an immobile object; it will still participate in collision
            //mSkels[0]->setImmobileState(true);

            // create a collision handler
            //mCollisionHandle = new rtql8::dynamics::ContactDynamics(mSkels, mTimeStep);
        }

        void World::updatePhysics() {
            mIntegrator.integrate(this, mTimeStep);
        }

        void World::setGravity(const Eigen::Vector3d& _gravity) {
            mGravity = _gravity;
        }

        void World::setTimeStep(double _timeStep) {
            mTimeStep = _timeStep;
        }

        const Eigen::Vector3d& World::getGravity(void) const {
            return mGravity;
        }

        double World::getTimeStep(void) const {
            return mTimeStep;
        }

        Eigen::VectorXd World::getState() {
            Eigen::VectorXd state(mIndices.back() * 2);
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i] * 2;
                int size = mDofs[i].size();
                state.segment(start, size) = mDofs[i];
                state.segment(start + size, size) = mDofVels[i];
            }
            return state;
        }

        void World::setState(Eigen::VectorXd newState) {
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i] * 2;
                int size = mDofs[i].size();
                mDofs[i] = newState.segment(start, size);
                mDofVels[i] = newState.segment(start + size, size);
            }
        }

        Eigen::VectorXd World::evalDeriv() {
            // TODO: !!!!
            // The root follows the desired acceleration and the rest of the
            // body follows dynamic equations. The root acceleration will impact
            // the rest of the body correctly. Collision or other external
            // forces will alter the root acceleration
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                if (mSkels[i]->getImmobileState()) {
                    // TODO: JS cannot understand that
                    //       why the index of mDofs is i.
                    mSkels[i]->setPose(mDofs[i], false, false);
                } else {
                    // TODO: JS cannot understand that
                    //       why the index of mDofs is i.
                    mSkels[i]->setPose(mDofs[i], false, false);
                    mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
                }
            }

            // compute contact forces
            //mCollisionHandle->applyContactForces();

            Eigen::VectorXd deriv = Eigen::VectorXd::Zero(mIndices.back() * 2);

            for (unsigned int i = 0; i < mSkels.size(); i++) {
                // skip immobile objects in forward simulation
                if (mSkels[i]->getImmobileState())
                    continue;
                int start = mIndices[i] * 2;
                int size = mDofs[i].size();
                Eigen::VectorXd qddot = mSkels[i]->getInvMassMatrix() * (-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mSkels[i]->getInternalForces());
                mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
                deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
                deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
            }

            return deriv;
        }

        void World::addSkeleton(dynamics::SkeletonDynamics* _skel) {
            mSkels.push_back(_skel);
        }

    } // namespace simulation
} // namespace rtql8
