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

#ifndef __KINNNET_H__
#define __KINNNET_H__

#include "KinHndlrConsts.h"

using namespace std;
//all functionality to process depth data from kinect
namespace rtql8{
    namespace kinect{
        class KinPercep{
            public:
                KinPercep(){}
                KinPercep(int numIns, bool _isBias) : inSize(numIns), isBias(_isBias), inWts(numIns), inActs(numIns), delta(0), outAct(0){
                    if (isBias) { outAct= 1;}
                    setInitRandWeight();
                }
                ~KinPercep(){}

                void setInitVals(int numIns, bool _isBias){
                    inSize = numIns;
                    isBias = _isBias;
                    inActs = vector<double>(inSize);
                    inWts = vector<double>(inSize);
                    setInitRandWeight();
                    delta = 0;
                    if (isBias) { outAct= 1;}
                }

                double round(double x){    return floor(x+0.5); }

                //sigmoid activation
                void calcActivation(){
                    double runningTotal = 0;
                    if(!isBias){
                        for (int i = 0; i<inSize; ++i){    runningTotal += inWts[i] * inActs[i]; }
                        outAct = round(1.0/(1.0 + exp(-1.0 * runningTotal)));
                    } else {
                        outAct = 1;
                    }
                }//calcActivation

                //deriv of sigmoid
                double calcDerivActivation(){
                    double result = 0.0, gz = 0, runningTotal = 0;
                    if(!(isBias)){
                        for (int i = 0; i<inSize; ++i){    runningTotal += inWts[i] * inActs[i];}
                        gz = (1.0/(1.0 + exp(-1.0 * runningTotal)));
                        result = (gz *(1-gz));
                    }
                    return result;
                }//calcDerivActivation

                double calcNewWeights(){
                    double totModAmt = 0, modAmt = 0;
                    for(int i = 0; i < inSize; ++i){
                        modAmt = (NNalpha * inActs[i] * delta);
                        inWts[i] += modAmt;
                        totModAmt += modAmt;
                    }//for
                    return totModAmt;
                }//calcNewWeights

                //enable loadin of weights and saving of weights
                void setWeights(vector<double>& wghts){  this->inWts = vector<double>(wghts);  }
                vector<double> getWeights(){    return this->inWts;}

                double getWeightError(int idx){    return delta*inWts[idx]; }

                void setInitRandWeight(){
                    std::default_random_engine gen;
                    std::uniform_real_distribution<double> dist(0.0,1.0);
                    std::uniform_int_distribution<int>intDist(0,1);
                    for (int i = 0; i< inSize; ++i){ inWts[i] = (dist(gen) + .0001) * (intDist(gen) ? 1 : -1); }//for            
                }

            public://variables
                int inSize;                         //number of perceps that feed into this percep
                bool isBias;                        //whether or not this is a bias percep
                vector<double> inWts, inActs;       //input weights and activations for this percep
                double delta, outAct;               //backprop delta, output activation
        };//class KinPercep

        class KinNNet{
            public:
                KinNNet(); 
                KinNNet(int _numInp, vector<int>& _numHidLayers, int _outs);
                ~KinNNet(){}
                void buildNet(int _numInp, vector<int>& _numHidLayers, int _outs);
                //update each perceptron of destlyr with vector of input activations from srcvec - remember to include input from bias percep
                void updateLayerActs(vector<int>& srcVec, vector<KinPercep>& destLyr){
                    for(int pidx = 0; pidx < destLyr.size(); ++pidx){  for(int inIdx = 0; inIdx < srcVec.size(); ++inIdx){    destLyr[pidx].inActs[inIdx] = srcVec[inIdx];  }  }//for each percep
                }//updateLayerActs

                //update each perceptron of destlyr with vector of input activations from srcvec - remember to include input from bias percep
                void updateLayerWgts(vector<int>& srcVec, vector<KinPercep>& destLyr){
                    for(int pidx = 0; pidx < destLyr.size(); ++pidx){  for(int inIdx = 0; inIdx < srcVec.size(); ++inIdx){    destLyr[pidx].inActs[inIdx] = srcVec[inIdx];  }  }//for each percep
                }//updateLayerWgts 

                void feedForward(vector<int>& _inVals);
                void randomizeWeights();
                void backPropLearning(vector<vector<int>>& inVec, vector<vector<int>>& outVec, double& totErr, double& avgWghtMod);
                vector<int> getOutputAsVector();

                void saveNet(string netName);
                void loadNet(string netName);

                bool isBuilt(){return isNowBuilt;}

            public:
                int numInp;                             //# of inputs, including bias percep, num f hi
                vector<vector<KinPercep>>hidLayers;     //hidden layers
                vector<KinPercep>outLayer;              //output layer
                bool isNowBuilt;
        };//class KinNet
    }//namespace kinect
}//namespace rtql8
#endif
