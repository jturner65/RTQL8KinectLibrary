/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): John Turner <jturner65@gatech.edu>
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

#ifndef KINSKELIKJNTCNSTRNT_H
#define KINSKELIKJNTCNSTRNT_H

#include "KinLibHelper.h"

namespace rtql8 {
    namespace kinematics {
        class Skeleton;
        class BodyNode;
    } // namespace kinematics

	namespace optimizer{
        class Var;
		class Constraint;
	}
}

using namespace rtql8::optimizer;

//represents mocap/kinect capture joint

namespace rtql8 {
    namespace kinect {


		class KinSkelIKJntCnstrnt : public Constraint {					
			public:
				KinSkelIKJntCnstrnt(std::vector<Var *>& var, kinematics::Skeleton* skel, kinematics::BodyNode* node, const Eigen::Vector3d& offset, const Eigen::Vector3d& val);

				virtual Eigen::VectorXd evalCon();
				virtual void fillJac(VVD, int){}
				virtual void fillJac(VVD, VVB, int);
				virtual void fillObjGrad(std::vector<double>&);

				void setName(string _name){name = _name;}
				void setName(const char* _name){name = string(_name);}
				string getName(){return name;}

				void setTarget(const Eigen::Vector3d& target);
				Eigen::Vector3d getTarget() const;

			protected:
				Eigen::Vector3d mTarget;
				Eigen::Vector3d mOffset;

				string name;

				kinematics::Skeleton* mSkel;
				kinematics::BodyNode* mNode;					
        };
    } // namespace optimizer
} // namespace rtql8
    
#endif // #ifndef KINSKELIKJNTCNSTRNT_H
