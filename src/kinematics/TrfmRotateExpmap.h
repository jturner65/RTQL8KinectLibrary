/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumitj83@gmail.com>, Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef KINEMATICS_TRFM_ROTATE_EXPMAP_H
#define KINEMATICS_TRFM_ROTATE_EXPMAP_H

#include "Transformation.h"

#include <cmath>
#include <cassert>

namespace rtql8 {
    namespace kinematics {

        class TrfmRotateExpMap: public Transformation {
        public:
            TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, const char* name = NULL);

            Eigen::Matrix4d getInvTransform();

            void applyGLTransform(renderer::RenderInterface* _ri) const;
            void computeTransform();
            Eigen::Matrix4d getDeriv(const Dof *);
            Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);

        protected:

        };

        /// \class TrfmRotateExpMap2
        /// \brief One-dof rotational transformation along an axis which is
        ///        constant.
        class TrfmRotateAxis1: public Transformation
        {
        public:
            /// \brief Constructor.
            /// \param[in] _axis Rotational axis which is constant.
            /// \param[in] q Rotation anlge which is variable.
            /// \param[in] name The name of this transformation.
            TrfmRotateAxis1(const Eigen::Vector3d& axis, Dof *q,
                              const char* name = NULL);

            // Documentation inherited.
            virtual Eigen::Matrix4d getInvTransform();

            // Documentation inherited.
            virtual void applyGLTransform(renderer::RenderInterface* _ri) const;

            // Documentation inherited.
            virtual void computeTransform();

            // Documentation inherited.
            virtual Eigen::Matrix4d getDeriv(const Dof* q1);

            // Documentation inherited.
            virtual Eigen::Matrix4d getSecondDeriv(const Dof* q1,
                                                   const Dof* q2);

        protected:
            /// \brief The rotational axis of this transformation.
            Eigen::Vector3d mAxis;
        };

    } // namespace kinematics
} // namespace rtql8

#endif // KINEMATICS_TRFM_ROTATE_EXPMAP_H
