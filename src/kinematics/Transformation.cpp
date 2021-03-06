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

#include "Transformation.h"

#include "utils/Misc.h" // For M_PI
#include "utils/UtilsMath.h"
#include "renderer/RenderInterface.h"

#include "Dof.h"
#include "Joint.h"

using namespace std;
using namespace Eigen;

namespace rtql8 {
    namespace kinematics {

        Transformation::Transformation()
            :mTransform(Matrix4d::Zero()) {
                strcpy(mName,"");
                mJoint=NULL;
                mVariable=true;
                mSkelIndex=-1;
                mDirty = true;
        }

        Transformation::~Transformation(){
            for(unsigned int i=0; i<mDofs.size(); i++) {
                delete mDofs[i];
            }
            mDofs.clear();
        }

        Matrix4d Transformation::getTransform() {
            if ( mDirty ) {
                computeTransform();
                mDirty = false;
            }
            return mTransform;
        }

        Matrix4d Transformation::getInvTransform() {
            if ( mDirty ) {
                computeTransform();
                mDirty = false;
            }
            return mTransform.inverse();
        }

        bool Transformation::isPresent(const Dof *q) const {
            for(unsigned int i=0; i<mDofs.size(); i++){
                if(q==mDofs[i]) return true;
            }
            return false;
        }

        void Transformation::applyTransform(Vector3d& _v) {
            _v= utils::xformHom(getTransform(), _v);
            Vector4d v4d( _v[0], _v[1], _v[2], 1 );
            v4d = getTransform() *v4d;
            _v = Vector3d( v4d[0], v4d[1], v4d[2] );
        }

        void Transformation::applyTransform(Matrix4d& _m) {
            _m = getTransform() *_m;
        }

        void Transformation::applyInvTransform(Vector3d& _v) {
            _v = utils::xformHom(getInvTransform(), _v);
        }

        void Transformation::applyInvTransform(Matrix4d& _m) {
            _m = getInvTransform() *_m;
        }

        void Transformation::applyDeriv(const Dof*_q, Vector3d& _v) {
            _v = utils::xformHom(getDeriv(_q), _v);
        }

        void Transformation::applyDeriv(const Dof*_q, Matrix4d& _m)  {
            _m = getDeriv(_q) *_m;
        }

        void Transformation::applySecondDeriv(const Dof*_q1, const Dof*_q2, Vector3d& _v) {
            _v = utils::xformHom(getSecondDeriv(_q1, _q2), _v);
        }

        void Transformation::applySecondDeriv(const Dof*_q1, const Dof*_q2, Matrix4d& _m) {
            _m = getSecondDeriv(_q1, _q2)*_m;
        }


    } // namespace kinematics
} // namespace rtql8
