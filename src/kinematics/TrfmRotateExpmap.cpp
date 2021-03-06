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

#include "TrfmRotateExpmap.h"
#include "Dof.h"
#include "utils/UtilsRotation.h"
#include "utils/UtilsMath.h"


using namespace std;
using namespace Eigen;

namespace rtql8 {
    namespace kinematics {

        TrfmRotateExpMap::TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, const char* _name){
            mDofs.resize(3);
            mDofs[0]=x;
            mDofs[1]=y;
            mDofs[2]=z;
            x->setTrans(this);
            y->setTrans(this);
            z->setTrans(this);
            mType = Transformation::T_ROTATEEXPMAP;
            if(_name!=NULL)
                strcpy(mName, _name);
            else
                strcpy(mName, "EXPMAP");
        }

        void TrfmRotateExpMap::computeTransform(){
            Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

            Matrix3d rot = utils::rotation::expMapRot(q);
            mTransform.setZero();
            mTransform.topLeftCorner(3,3) = rot;
            mTransform(3, 3) = 1.0;
        }

        Matrix4d TrfmRotateExpMap::getInvTransform(){
            if(mDirty){
                computeTransform();
                mDirty=false;
            }
            return mTransform.transpose();
        }


        Matrix4d TrfmRotateExpMap::getDeriv(const Dof *d){
            Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

            // derivative wrt which dof
            int j=-1;
            for(unsigned int i=0; i<mDofs.size(); i++) if(d==mDofs[i]) j=i;
            assert(j!=-1);
            assert(j>=0 && j<=2);

            Matrix3d R = utils::rotation::expMapRot(q);
            Matrix3d J = utils::rotation::expMapJac(q);
            Matrix3d dRdj = utils::makeSkewSymmetric(J.col(j))*R;

            Matrix4d dRdj4d = Matrix4d::Zero();
            dRdj4d.topLeftCorner(3,3) = dRdj;

            return dRdj4d;
        }

        Matrix4d TrfmRotateExpMap::getSecondDeriv(const Dof *q1, const Dof *q2){
            Vector3d q(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());

            // derivative wrt which mDofs
            int j=-1, k=-1;
            for(unsigned int i=0; i<mDofs.size(); i++) {
                if(q1==mDofs[i]) j=i;
                if(q2==mDofs[i]) k=i;
            }
            assert(j!=-1);
            assert(k!=-1);
            assert(j>=0 && j<=2);
            assert(k>=0 && k<=2);

            Matrix3d R = utils::rotation::expMapRot(q);
            Matrix3d J = utils::rotation::expMapJac(q);
            Matrix3d Jjss = utils::makeSkewSymmetric(J.col(j));
            Matrix3d Jkss = utils::makeSkewSymmetric(J.col(k));
            Matrix3d dJjdkss = utils::makeSkewSymmetric(utils::rotation::expMapJacDeriv(q, k).col(j));

            Matrix3d d2Rdidj = (Jjss*Jkss + dJjdkss)*R;

            Matrix4d d2Rdidj4 = Matrix4d::Zero();
            d2Rdidj4.topLeftCorner(3,3) = d2Rdidj;

            return d2Rdidj4;
        }

        void TrfmRotateExpMap::applyGLTransform(renderer::RenderInterface* _ri) const{
            Vector3d v(mDofs[0]->getValue(), mDofs[1]->getValue(), mDofs[2]->getValue());
            double theta = v.norm();
            Vector3d vhat = Vector3d::Zero();
            if(!utils::isZero(theta)) {
                vhat= v/theta;
                _ri->rotate(vhat, theta * 180 / M_PI);
            }

        }

        //////////////////////////////////////////////////
        TrfmRotateAxis1::TrfmRotateAxis1(const Vector3d& _axis,
                                             Dof *q, const char* _name)
        {
            // TODO: need to test
            mAxis = _axis;
            mDofs.resize(1);
            mDofs[0] = q;
            q->setTrans(this);
            mType = Transformation::T_ROTATEEXPMAP2;
            if(_name != NULL)
                strcpy(mName, _name);
            else
                strcpy(mName, "EXPMAP2");
        }

        //////////////////////////////////////////////////
        void TrfmRotateAxis1::computeTransform()
        {
            // TODO: need to test
            Matrix3d rot = utils::rotation::expMapRot3(mAxis,
                                                       mDofs[0]->getValue());
            mTransform.setZero();
            mTransform.topLeftCorner(3,3) = rot;
            mTransform(3, 3) = 1.0;
        }

        //////////////////////////////////////////////////
        Matrix4d TrfmRotateAxis1::getInvTransform()
        {
            // TODO: need to test
            if(mDirty)
            {
                computeTransform();
                mDirty = false;
            }
            return mTransform.transpose();
        }

        //////////////////////////////////////////////////
        Matrix4d TrfmRotateAxis1::getDeriv(const Dof *d)
        {
            // TODO: need to test
            // d(Exp(Aq)) / dq = A*Exp(Aq)
            Matrix4d ret = Matrix4d::Zero();

            if (d == mDofs[0])
            {
                Matrix3d R = utils::rotation::expMapRot3(mAxis,
                                                         mDofs[0]->getValue());
                Matrix3d A = utils::makeSkewSymmetric(mAxis);

                ret.topLeftCorner(3,3) = A*R;
            }

            return ret;
        }

        //////////////////////////////////////////////////
        Matrix4d TrfmRotateAxis1::getSecondDeriv(const Dof *q1, const Dof *q2)
        {
            // TODO: need to test
            // d^2(Exp(Aq)) / d^2q^2 = A^2*Exp(Aq)
            Matrix4d ret = Matrix4d::Zero();

            if (q1 == mDofs[0] && q2 == mDofs[0])
            {
                Matrix3d R = utils::rotation::expMapRot3(mAxis,
                                                         mDofs[0]->getValue());
                Matrix3d A = utils::makeSkewSymmetric(mAxis);

                ret.topLeftCorner(3,3) = A*A*R;
            }

            return ret;
        }

        //////////////////////////////////////////////////
        void TrfmRotateAxis1::applyGLTransform(
                renderer::RenderInterface* _ri) const
        {
            // TODO: need to test
            double theta = mDofs[0]->getValue();
            if (!utils::isZero(theta))
            {
                _ri->rotate(mAxis, theta * 180 / M_PI);
            }

        }

    } // namespace kinematics
} // namespace rtql8
