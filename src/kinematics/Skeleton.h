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

#ifndef KINEMATICS_SKELETON_H
#define KINEMATICS_SKELETON_H

#include <vector>
#include <Eigen/Dense>
#include "renderer/RenderInterface.h"

namespace rtql8 {
    namespace kinematics {

        class Transformation;
        class Marker;
        class Joint;
        class BodyNode;
        class Dof;

        class Skeleton {
        public:
            Eigen::VectorXd mCurrPose;
            BodyNode* mRoot;


            Skeleton();
            virtual ~Skeleton();

            virtual BodyNode* createBodyNode(const char* const name = NULL);
            void addMarker(Marker *_h);
            // TODO: (in test)
            // RTQL8 loads skeleton according to following order: link, joint,
            // link, joint, link, .... For this reason, addNode() function add
            // the parent joint by calling addJoint(). However, Gazebo loads
            // model (skeleton) in following order: link, link, ..., joint,
            // joint, .... Therefore, in addNode() function, we sill don't
            // know the parent joint.
            // To resolve this problem, JS add the optional parameter
            // _addParentJoint. In RTQL8, we set the parameter as true, and, in
            // Gazebo, we set the parameter as false. When we create joint, we
            // must add the joint manually to skeleton by calling addJoint()
            // function.
            void addNode(BodyNode *_b, bool _addParentJoint = true);
            void addJoint(Joint *_j);
            void addDof(Dof *_d);
            void addTransform(Transformation *_t);

            // init the model after parsing
            void initSkel();

            // inline access functions
            inline int getNumDofs() { return mDofs.size(); }
            inline int getNumNodes() { return mNodes.size(); }
            inline int getNumMarkers() { return mMarkers.size(); }
            inline int getNumJoints(){return mJoints.size();}
            inline Dof* getDof(int _i) { return mDofs[_i]; }
            inline BodyNode* getNode(int _i) { return mNodes[_i]; }
            inline BodyNode* getRoot() { return mRoot; }
            BodyNode* getNode(const char* const name);
            int getNodeIndex(const char* const name);
            inline Marker* getMarker(int _i) { return mMarkers[_i]; }
            inline double getMass() { return mMass; }
            Eigen::Vector3d getWorldCOM();

            virtual void setPose(const Eigen::VectorXd&, bool bCalcTrans = true, bool bCalcDeriv = true);
            virtual void setPose(const std::vector<double>&, bool bCalcTrans = true, bool bCalcDeriv = true);

            void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const;
            void drawMarkers(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true ) const;

            //void setPose(const Eigen::VectorXd& _pose);
            //void setPose(const std::vector<double>& _pose);
            void getPose(Eigen::VectorXd& _pose);
            void getPose(std::vector<double>& _pose);
            Eigen::MatrixXd getJacobian(BodyNode* _bd, Eigen::Vector3d _localOffset);

        protected:
            std::vector<Marker*> mMarkers;
            std::vector<Dof*> mDofs;
            std::vector<Transformation*> mTransforms;
            std::vector<BodyNode*> mNodes;
            std::vector<Joint*> mJoints;

            double mMass;
        };

    } // namespace kinematics
} // namespace rtql8

#endif // #ifndef KINEMATICS_SKELETON_H

