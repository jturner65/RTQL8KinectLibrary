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

#ifndef __KINSKELIKSOLVER_H__
#define __KINSKELIKSOLVER_H__

#include "KinSkelIKJntCnstrnt.h"

using namespace rtql8::optimizer;
using namespace rtql8::kinematics;

//forward declarations of classes used
namespace rtql8 {
    namespace kinematics {
        class Skeleton;
    }
}

namespace rtql8 {
    namespace optimizer {
        class ObjectiveBox;
        class Var;
    }
}
namespace rtql8 {
	namespace kinect{
		class KinSkelIKSolver{
		public:
			KinSkelIKSolver();
			~KinSkelIKSolver();

			void initSolver(rtql8::kinematics::Skeleton* _skel);
			void Solve(double fq);
			//void calcJacobianRow(int i);	//global Jacobian Solver for 1 row (constraint - actually a column in Jt)
			void buildWeights(Eigen::VectorXd& _wghts);			//vector of constraint/handle - based weights, to determine which handle-marker pairs should be considered more important and which are less important
			double calcFq();

			int getNumHandles(){return numHandles;}
			bool isReadyToSolve();

			void setMarkerData(vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& _mData);
			void setNumHandles(int _num){numHandles = _num;}
			void setMoCapLoaded(bool _mcLoaded);
			void setKinDataLoaded(bool _posDataLoaded);
			void setCurGFunc(int _g);
			double getDofMass(int i);

			void resetSolver();
			Eigen::VectorXd gradQ(){return Eigen::VectorXd(newQ-oldQ);}

			void DisplayWeights();	//debug

			//variables
			bool runSolver;													//whether to run the solver or not

	private : 
			bool moCapLoaded;												//whether mocap/kinect positiondata has been loaded/readied for use by ik solver
			int numHandles;													//number of handles the model for this solver has
			int numMarkers;													//number of mocap markers
			int numDofs;													//number of dofs for model this solver is solving for
			int numNodes;													//number of body nodes in skeleton
			double minDistToIK;												//minimum distance to attempt to IK to : prtrb amt * num markers

			//variables holding skeleton and optimisation info
		    rtql8::kinematics::Skeleton* mSkel;								//skeleton read in from file
			rtql8::optimizer::ObjectiveBox* mObjBox;						//holds all constraints (markers from kinect)
			vector<rtql8::optimizer::Var*> mDofVars;						//vars associated with dofs - holds value, min and max

			Eigen::VectorXd C, weights, oldQ, newQ, massVals, transNodeMass;

		};//KinSkelIKSolver
	}//namespace KinectHandlerNS
}//namespace RTQL8

#endif