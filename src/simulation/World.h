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

#ifndef SIMULATION_WORLD_H
#define SIMULATION_WORLD_H

#include <vector>
#include <Eigen/Dense>
//#include <Eigen/Cholesky>
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "dynamics/SkeletonDynamics.h"

namespace rtql8 {
    namespace simulation {

        class World : public integration::IntegrableSystem {

        public:
            /// \brief .
            World();

            /// \brief .
            virtual ~World();

            /// \brief .
            void init();

            /// \brief .
            void updatePhysics();

            /// \brief .
            void setGravity(const Eigen::Vector3d& _gravity);

            /// \brief .
            void setTimeStep(double _timeStep);

            /// \brief .
            const Eigen::Vector3d& getGravity(void) const;

            /// \brief .
            double getTimeStep(void) const;

            /// \brief .
            virtual Eigen::VectorXd getState();

            /// \brief .
            virtual void setState(Eigen::VectorXd);

            /// \brief .
            virtual Eigen::VectorXd evalDeriv();

            /// \brief .
            void addSkeleton(dynamics::SkeletonDynamics* _skel);

        protected:
            /// \brief .
            std::vector<dynamics::SkeletonDynamics*> mSkels;  /// Array of skeletons

            /// \brief .
            std::vector<Eigen::VectorXd> mDofVels;

            /// \brief .
            std::vector<Eigen::VectorXd> mDofs;

            /// \brief .
            integration::EulerIntegrator mIntegrator;

            /// \brief .
            Eigen::Vector3d mGravity;

            /// \brief .
            std::vector<int> mIndices;

            /// \brief .
            double mTimeStep;

            /// \brief .
            bool running;

            /// \brief .
            int frame;
        };

    } // namespace simulation
} // namespace rtql8

#endif // #ifndef SIMULATION_WORLD_H
