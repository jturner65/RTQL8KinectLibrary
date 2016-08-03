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

#include "KinHndlrConsts.h"
#include "KinSkelIKSolver.h"

using namespace std;
using namespace rtql8::utils;
using namespace rtql8::optimizer;
using namespace rtql8::kinematics;

namespace rtql8{
    namespace kinect{

        KinSkelIKSolver::KinSkelIKSolver() : runSolver(false), moCapLoaded(false), numHandles(0), numMarkers(0), numDofs(0), numNodes(0), 
                                             C(), weights(), oldQ(), newQ(), massVals(), transNodeMass(){	
        } 

        KinSkelIKSolver::~KinSkelIKSolver(){
            delete mObjBox;
        }

        //initialize solver variables
        void KinSkelIKSolver::initSolver(rtql8::kinematics::Skeleton* _skel){
            mSkel = _skel;
            numDofs = mSkel->getNumDofs();
            numHandles = mSkel->getNumMarkers();										//ik handles from skel

            for (int i = 0; i < numDofs; i++) {
                Var *v = new Var(mSkel->getDof(i)->getValue(), mSkel->getDof(i)->getMin(), mSkel->getDof(i)->getMax());
                mDofVars.push_back(v);
            }
            //objective box for optimizer - initialize it with ik handles from skeleton
            mObjBox = new ObjectiveBox(numDofs);
	
            Marker* mk;
            BodyNode *node;
            Eigen::Vector3d offset,target;												//distance from node to marker
            KinSkelIKJntCnstrnt* pos;													//marker to ik to

            for(int i = 0; i < numHandles; ++i){										//start with IK markers at same location as handles on skeleton
                mk = mSkel->getMarker(i);
                node = mk->getNode();
                //offset = Eigen::Vector3d(0, 0, 0);
                //target = xformHom(node->getWorldTransform(), offset);
                offset = mk->getLocalCoords();
                target = mk->getWorldCoords();		
                //clog<<"KinSkelIKSolver : marker["<<i<<"] name : "<<mk->getName()<<" Location : "<<target<<std::endl;
                //clog<<std::endl;
                pos = new KinSkelIKJntCnstrnt(mDofVars, mSkel, node, offset, target);
                pos->setName(mk->getName());
                mObjBox->add(pos);
            }//for each marker associated with this skel
            numMarkers = numHandles;													//markers for IK built as clones of handles
            oldQ=Eigen::VectorXd(numDofs);												//global vectors holding most recent previous and current pose vectors
            newQ=Eigen::VectorXd(numDofs);		
            massVals = Eigen::VectorXd(numDofs);
            transNodeMass = Eigen::VectorXd(mSkel->getNumNodes());
            minDistToIK = KinIK_minPrtrb * numMarkers * .5f;
            clog<<"\tKinSkelIKSolver : # handles : "<<numHandles<<std::endl;
            clog<<"\tKinSkelIKSolver : # markers : "<<numMarkers<<std::endl;

            weights = Eigen::VectorXd::Ones(numHandles);//buildWeights();
		
        }//initSolver

        double KinSkelIKSolver::getDofMass(int i){			return 0.0;	}//for solving weighted optimisation - replace with actual mass values

        //calculate cost function - returns sum of squared dists 
        double KinSkelIKSolver::calcFq(){
            double fq = 0, tmp = 0;
            Eigen::Vector3d moCapLoc;
            for(int i=0; i<numHandles; ++i){
                moCapLoc = ((KinSkelIKJntCnstrnt*)mObjBox->getConstraint(i))->getTarget();
                if (vec3_zero == moCapLoc){						tmp = 0;				}
                else {			
                    //tmp = weights[i] * (mSkel->getMarker(i)->getWorldCoords() - moCapLoc).squaredNorm();				
                    tmp = weights[i] * (mSkel->getMarker(i)->getWorldCoords() - moCapLoc).norm();				
                }
                fq += tmp;
            }
            return fq;
        }//calcFq

        //set marker data from kinect for IK
        void KinSkelIKSolver::setMarkerData(vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& _mData){
            for(int i = 0; i< _mData.size(); ++i){		
                //clog<<"cnstrnt["<<i<<"] name : "<<((KinSkelIKJntCnstrnt*)mObjBox->getConstraint(i))->getName()<<std::endl;
                ((KinSkelIKJntCnstrnt*)mObjBox->getConstraint(i))->setTarget(_mData[i]);
            }
            moCapLoaded = true;
        }//setMarkerData

        //whether or not this solver is ready to go
        bool KinSkelIKSolver::isReadyToSolve(){
            if((nullptr != mSkel) && (0 != mObjBox->getNumConstraints()) ){			return moCapLoaded;}
            else return false;
        }//isReadyToSolve

        //reset/reinitialize this solver
        void KinSkelIKSolver::resetSolver(){
            numHandles = 0;	
            numDofs = 0;
            numMarkers = 0;
            numNodes = 0;
            runSolver = false; 
            moCapLoaded = false;
            delete mObjBox;
        }//resetSolver

        //entering this from main.cpp - fq is being calcuated in main, and the loop is being controlled there. 
        //we know as we enter that fq > KinIK_eps for current dof config
        void KinSkelIKSolver::Solve(double fq){
            Eigen::VectorXd _oldq, _newq;//, partialF;
            Eigen::VectorXd q(numDofs);
            bool improving = true;
            int count = 0;
            double fqOld = fq, fqNew;

            int nDof =  mSkel->getNumDofs();
            mObjBox->evalObjGrad();
            Eigen::VectorXd newPose(nDof);
            for (int j = 0; j < nDof; j++){
                newPose[j] = mSkel->getDof(j)->getValue() - KinIK_minPrtrb * mObjBox->mObjGrad[j];
            }
            Eigen::VectorXd oldPose = Eigen::VectorXd (newPose);

            while(improving){		
                mSkel->setPose(newPose, true, false);			
                fqNew = calcFq();									//get new distance from appropriate pose with newly set dofs
                if((fqNew > fqOld) || (fqNew < KinIK_eps)){			//if our newly set dofs don't get us any better, or if our newly set dofs setting is closer than eps to correct
                    improving = false;				
                } else {											//fqNew < fqOld - we're improving, try to improve some more
                    fqOld = fqNew;									//fQ of newly applied dofs
                    oldPose = newPose;								//keep to reset dofs to most recent -good- q
                    for(int qi = 0; qi < q.size(); ++qi){
                        newPose[qi] = mSkel->getDof(qi)->getValue() - KinIK_minPrtrb * mObjBox->mObjGrad[qi];
                    }

                }//if getting worse or fqNew < KinIK_eps, else

                // A naive way for bounding joint angles
                for (int i = 0; i < mSkel->getNumDofs(); i++) {
                    rtql8::kinematics::Dof* dof = mSkel->getDof(i);
                    double lower = dof->getMin();
                    double upper = dof->getMax();
                    double value = dof->getValue();
                    if (value < lower) value = lower;
                    if (value > upper) value = upper;
                    dof->setValue(value);
                }
                
                ++count;
            }//while

            mSkel->setPose(oldPose, true, true);
            //newQ = _newq;
            //oldQ = _oldq;
        }//Solve


        //build weight vector 
        void KinSkelIKSolver::buildWeights(Eigen::VectorXd& _wghts){
            //make initial weights be related to masses of transform nodes that handles are connected to
            weights = Eigen::VectorXd::Ones(numHandles);				//weights array - set to all 1's now - can change to weigh more distant handles from their markers higher than those that are closer
        }

    }//namespace kinect
}//namespace rtql8
