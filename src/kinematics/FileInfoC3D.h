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

#ifndef KINEMATICS_FILEINFO_C3D_H
#define KINEMATICS_FILEINFO_C3D_H

#include <vector>
#include <Eigen/Dense>

#include "utils/EigenHelper.h"

namespace rtql8 {
    namespace kinematics {
        class FileInfoC3D {

        public:
            FileInfoC3D();
            virtual ~FileInfoC3D(){}

            inline int getNumMarkers() const { return mNumMarkers; }
            inline int getNumFrames() const { return mNumFrames; }
            inline double getFPS() const { return mFPS; }

            inline Eigen::Vector3d getDataAt(int _frame, int _idx) const { return mData.at(_frame).at(_idx); } ///< Note: not checking index range
            inline void addData(const EIGEN_V_VEC3D& _data) { mData.push_back(_data); }

            virtual bool loadFile(const char*);
            virtual bool saveFile(const char*, int _start, int _end, double _sampleRate = 1); ///< Note: down sampling not implemented yet

        protected:
            int mNumMarkers;
            int mNumFrames;
            EIGEN_VV_VEC3D mData;
            double mFPS;
            char mFileName[256]; // change to string?
        };
    } // namespace kinematics
} // namespace rtql8

#endif // #ifndef KINEMATICS_FILEINFO_C3D_H

