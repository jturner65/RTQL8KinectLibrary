/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
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

#ifndef UTILS_UTILSROTATION_H
#define UTILS_UTILSROTATION_H

// External Libraries
#include <Eigen/Dense>
// Local Headers
#include "Misc.h"

namespace rtql8 {
    namespace utils {
        namespace rotation {

            enum RotationOrder {UNKNOWN, XYZ, XZY, YZX, YXZ, ZXY, ZYX};

            // forms the Quaterniond from a rotation matrix
            Eigen::Quaterniond matrixToQuat(Eigen::Matrix3d& m);
            Eigen::Matrix3d quatToMatrix(Eigen::Quaterniond& q);

            Eigen::Quaterniond expToQuat(Eigen::Vector3d& v);
            Eigen::Vector3d quatToExp(Eigen::Quaterniond& q);


            // Note: xyz order means matrix is Rz*Ry*Rx i.e a point as
            // transformed as Rz*Ry*Rx(p)
            // coord sys transformation as in GL will be written as glRotate(z);
            // glRotate(y); glRotate(x)
            Eigen::Vector3d matrixToEuler(Eigen::Matrix3d& m,
                                          RotationOrder _order);
            Eigen::Matrix3d eulerToMatrix(Eigen::Vector3d& v,
                                          RotationOrder _order);

            Eigen::Matrix3d eulerToMatrixX(double x);
            Eigen::Matrix3d eulerToMatrixY(double y);
            Eigen::Matrix3d eulerToMatrixZ(double z);

            Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q,
                                        const Eigen::Vector3d& pt);
            Eigen::Vector3d rotatePoint(const Eigen::Quaterniond& q,
                                        double x, double y, double z);

            // quaternion stuff
            Eigen::Matrix3d quatDeriv(const Eigen::Quaterniond& q, int el);
            Eigen::Matrix3d quatSecondDeriv(const Eigen::Quaterniond& q,
                                            int el1, int el2);

            // compute expmap stuff
            /// computes the Rotation matrix from a given expmap vector
            Eigen::Matrix3d expMapRot(const Eigen::Vector3d &_expmap);
            /// (in test for speed-up)
            /// computes the Rotation matrix from a given expmap vector
            Eigen::Matrix3d expMapRot2(const Eigen::Vector3d &_expmap);
            /// (in test for speed-up)
            /// computes the Rotation matrix from a given expmap vector
            Eigen::Matrix3d expMapRot3(const Eigen::Vector3d &_expmap,
                                       const double theta);

            /// computes the Jacobian of the expmap
            Eigen::Matrix3d expMapJac(const Eigen::Vector3d &_expmap);

            /// computes the time derivative of the expmap Jacobian
            Eigen::Matrix3d expMapJacDot(const Eigen::Vector3d &_expmap,
                                         const Eigen::Vector3d &_qdot);
            /// computes the derivative of the Jacobian of the expmap wrt to
            /// _qi indexed dof; _qi \in {0,1,2}
            Eigen::Matrix3d expMapJacDeriv(const Eigen::Vector3d &_expmap,
                                           int _qi);

        } // namespace rotation
    } // namespace utils
} // namespace rtql8

#endif // #ifndef UTILS_UTILSROTATION_H

