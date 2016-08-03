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
#include "KinNNet.h"

using namespace std;
//all functionality to process depth data from kinect
namespace rtql8{
    namespace kinect{

			KinNNet::KinNNet(): numInp(0), hidLayers(0), outLayer(0), isNowBuilt(false){} 
			KinNNet::KinNNet(int _numInp, vector<int>& _numHidLayers, int _outs): numInp(_numInp+1), hidLayers(_numHidLayers.size()), outLayer(_outs), isNowBuilt(false){
                buildNet(_numInp,_numHidLayers, _outs);
				randomizeWeights();                                                           //initial random weights for all perceps
			}//cnstrctr

            //build neural net with passed params
            void KinNNet::buildNet(int _numInp, vector<int>& _numHidLayers, int _outs){
                numInp = _numInp + 1;
				int numHidLyrs = _numHidLayers.size(),                                  //
					prevNumLayers = numInp;                                             //+1 for bias -- input vector needs a 1 in 0th idx
				hidLayers = vector<vector<KinPercep>>(numHidLyrs);  

					//build hidden layers
				for(int idx = 0; idx < numHidLyrs; ++idx){                              //for each hidden layer
					vector<KinPercep> tmpLyr(_numHidLayers[idx]+1);                     //+1 for bias input
					tmpLyr[0] = KinPercep(prevNumLayers, true);                         //set first percep of each hidden layer to be a bias percep
					for(int pidx = 1; pidx < _numHidLayers[idx]+1; ++pidx){  
						KinPercep tmpPrcp = KinPercep(prevNumLayers, false); 
						tmpLyr[pidx] = tmpPrcp;      
					}                                                                   //build each percep
					hidLayers[idx] = tmpLyr;
					prevNumLayers = tmpLyr.size();                                      //number of perceps on this layer == number of inputs on next layer + 1 for bias
				}

                outLayer = vector<KinPercep>(_outs);                                    //output layer
					//build output layer
				for(int idx = 0; idx < _outs; ++idx){
					KinPercep tmpPrcp = KinPercep(prevNumLayers, false);                //initialize each percep with appropriate # of input weights
					outLayer[idx] = tmpPrcp;   
				}
                isNowBuilt = true;
            }//buildNet

			//pass input vector (without bias, bias is added here)
			void KinNNet::feedForward(vector<int>& _inVals){                                         //feed input through network
				vector<int> inputs(1);
				inputs[0] = 1;                                                              //bias
				for(vector<int>::iterator iter = _inVals.begin(); iter != _inVals.end(); ++iter){ inputs.push_back(*iter); }        //build input vector

				updateLayerActs(inputs, hidLayers[0]);                              //update inputs for hidden layer
				for(int i = 1; i < hidLayers.size(); ++i){                      //update the rest of the hidden layers
					inputs = vector<int>();                                     //remake input to next layer
					for(vector<KinPercep>::iterator iter = hidLayers[i-1].begin(); iter != hidLayers[i-1].end(); ++iter){           //build output activations for each prev layer percep, and then build input vector to next layer
						iter->calcActivation();                                 //calculate prev layer's perceptron's activation
						inputs.push_back(iter->outAct);                         //build input to current layer 
					}
					updateLayerActs(inputs, hidLayers[i]);
				}//for each hidden layer - need to propagate to output
				//output
				for(vector<KinPercep>::iterator iter = hidLayers[hidLayers.size()-1].begin(); iter != hidLayers[hidLayers.size()-1].end(); ++iter){ 
					iter->calcActivation();                             //calculate prev layer's perceptron's activation
					inputs.push_back(iter->outAct);                     //build input to current layer 
				}
				updateLayerActs(inputs, outLayer);
				for(vector<KinPercep>::iterator iter = outLayer.begin(); iter != outLayer.end(); ++iter){ iter->calcActivation(); }     //calc output layer activation
			}//feedforward

			//randomizes the weights of all the perceps in this net
			void KinNNet::randomizeWeights(){
				for(int lyrIdx = 0 ; lyrIdx < hidLayers.size(); --lyrIdx){                //step back to first hidden layer
					for(int pIdx = 0; pIdx < hidLayers[lyrIdx].size(); ++pIdx){
						hidLayers[lyrIdx][pIdx].setInitRandWeight();
					}
				}//for all hidden layers
				for(int opIdx = 0; opIdx < outLayer.size(); ++opIdx){
					outLayer[opIdx].setInitRandWeight();
				}//for each output percep
			}//rndWeights

			//feed all training examples and output classifications for one pass through back propagation algorithm 
			void KinNNet::backPropLearning(vector<vector<int>>& inVec, vector<vector<int>>& outVec, double& totErr, double& avgWghtMod){
				totErr = 0;
				avgWghtMod = 0;
				int numExamples = inVec.size();                                                 //# of examples - inVec.size() == outVec.size()
				double errVal = 0, weightMod = 0;
				int numWeights = 0;

				vector<int> rndIdx = vector<int>(numExamples,-1);                               //list of digits to use as idx's : set to -1 all empty spots
				rndIdx[0] = 0;                                                                  //initialize the first element to be size of array.  no legal element has 0
				srand (time(NULL));
				for(int i = numExamples-1; i > 0; --i){                                         //fisher-yates algorithm for list of non-duped random numbers, modified to handle empty locations in ara 
					int j = rand() % numExamples;
					if(rndIdx[j] != -1){ if(rndIdx[i] != -1){        std::swap(rndIdx[i],rndIdx[j]);    } else {   rndIdx[i] = rndIdx[j];  rndIdx[j] = i;  }}                           
					else { if(rndIdx[i] !=-1){       rndIdx[j] = rndIdx[i];     rndIdx[i] = j;} else {  rndIdx[i] = j;   rndIdx[j] = i;   }}
				}
				//for(int i = 0; i<numExamples; ++i){ clog<<"idx :"<<i<<" | "<<rndIdx[i]<<std::endl;}

				for(int idx = 0; idx < numExamples; ++idx){
					int exIdx = rndIdx[idx];                                                    //get random idx to decide which input to feed each iteration
					feedForward(inVec[exIdx]);                                                  //propagate inputs to outputs
					for(int outIdx = 0; outIdx < outLayer.size(); ++outIdx){                    //calculate output errors and backpropagate to modify error values held in each percep
						errVal += ((outVec[exIdx][outIdx] - outLayer[outIdx].outAct) * (outVec[exIdx][outIdx] - outLayer[outIdx].outAct));
						outLayer[outIdx].delta = outLayer[outIdx].calcDerivActivation() * (outVec[exIdx][outIdx] - outLayer[outIdx].outAct);                        
					}

					int lstHidLyrSz = hidLayers[hidLayers.size()-1].size();
					for(int pIdx = 0; pIdx < lstHidLyrSz; ++pIdx){                              //handle last hidden layer, fed by output layer
						double totWeightErr = 0;
						for(int wIdx = 0; wIdx < outLayer.size(); ++wIdx){
							totWeightErr += outLayer[wIdx].getWeightError(pIdx);
						}//for each percep in output layer
						hidLayers[hidLayers.size()-1][pIdx].delta = hidLayers[hidLayers.size()-1][pIdx].calcDerivActivation() * totWeightErr;                        
					}//for each percep in last hidden layer

					for(int lyrIdx = hidLayers.size()-2; lyrIdx > -1; --lyrIdx){                //step back to first hidden layer
						for(int pIdx = 0; pIdx < hidLayers[lyrIdx].size(); ++pIdx){
							double totWeightErr = 0;
							for(int wIdx = 0; wIdx < hidLayers[lyrIdx+1].size(); ++wIdx){
								totWeightErr += hidLayers[lyrIdx+1][wIdx].getWeightError(pIdx);
							}//for each percep in subsequent layer
							hidLayers[lyrIdx][pIdx].delta = hidLayers[lyrIdx][pIdx].calcDerivActivation() * totWeightErr;
						}//for each percep
					}//for each hidden layer

					for(int lyrIdx = 0; lyrIdx < hidLayers.size(); ++lyrIdx){           //update every weight in network using calculated deltas
						double wgtModTmp = 0;
						for(int pIdx = 0; pIdx < hidLayers[lyrIdx].size(); ++pIdx){
							numWeights += hidLayers[lyrIdx][pIdx].inSize;               //update number of weights
							wgtModTmp = hidLayers[lyrIdx][pIdx].calcNewWeights();
							weightMod += abs(wgtModTmp);
						}//for each hidden layer percep
					}//for each hidden layer

					//for each output layer percep - re calc weights
					for(int opIdx = 0; opIdx < outLayer.size(); ++opIdx){
						numWeights += outLayer[opIdx].inSize;               //update number of weights
						double wgtModTmp = outLayer[opIdx].calcNewWeights();
						weightMod += abs(wgtModTmp);
					}
				}//for each example                                           //calculate final error and avg weight mod values
				totErr = errVal/(1.0 * numExamples * outVec[0].size());            //number of examples x length of output vector
				avgWghtMod = weightMod/(1.0 * numWeights);                
			}//backPropLearning

			vector<int> KinNNet::getOutputAsVector(){
				vector<int> result(outLayer.size());
				//for(vector<KinPercep>::iterator iter = outLayer.begin(); iter != outLayer.end(); ++iter){ result.push_back(iter->outAct);  } //build result to hold output activation in each vector location
				for(int iter = 0; iter < outLayer.size(); ++iter){ result[iter] = outLayer[iter].outAct;  } //build result to hold output activation in each vector location
				return result;
			}//getOutputAsVector

            //save net weights to file
            void KinNNet::saveNet(string netName){
                std::ofstream netFile;
                netFile.open(netName.c_str(), std::ofstream::out | std::ofstream::app);
                //first save net config
				int numLayers = hidLayers.size();
                netFile<<"# Inputs (not count bias) : "<<(numInp-1)<<" # HidLyrs : "<<numLayers<<"# perceps per layer:";
				for(int hlyr = 0; hlyr < numLayers; ++hlyr){                
                    netFile<<" lyr (no bias) : "<<hlyr<<" : "<<(hidLayers[hlyr].size()-1);
                }
                netFile<<" # Outputs : "<<outLayer.size()<<endl;
                //rewrite with just data, for ease of parsing
                netFile<<(numInp-1)<<"|"<<numLayers<<"|";
				for(int hlyr = 0; hlyr < numLayers; ++hlyr){                
                    netFile<<(hidLayers[hlyr].size()-1)<<"|";
                }
                netFile<<outLayer.size()<<"|"<<endl;
                netFile<<"-------weights per layer per percep per input to percep : "<<endl;
                netFile<<"-------Hidden Layers : "<<endl;
				for(int hlyr = 0; hlyr < numLayers; ++hlyr){
					for(int pIdx = 0; pIdx < hidLayers[hlyr].size(); ++pIdx){
    				    netFile<<"hl: "<<hlyr<<" p: "<<pIdx<<" : "<<endl;
                        for(int wIdx = 0; wIdx < hidLayers[hlyr][pIdx].inWts.size(); ++wIdx){
    						netFile<<hidLayers[hlyr][pIdx].inWts[wIdx]<<"|";
                        }
                        netFile<<endl;
					}//for all perceps
				}//for all hidden layers
                netFile<<"-------Output Layer : "<<endl;
				for(int pIdx = 0; pIdx < outLayer.size(); ++pIdx){
                    netFile<<"out : p : "<<pIdx<<endl;
		            for(int wIdx = 0; wIdx < outLayer[pIdx].inWts.size(); ++wIdx){
    				    netFile<<outLayer[pIdx].inWts[wIdx]<<"|";
                    }
                    netFile<<endl;
				}//for all perceps
                netFile.close();
            }//saveNet

            //load net weights from file
            void KinNNet::loadNet(string netName){
                std::ifstream netFile;
                netFile.open(netName.c_str(), std::ofstream::in);
                string netLayoutStr, valStr;
                //first line is info line
                std::getline(netFile,netLayoutStr);
                clog<<"netLayoutStr : "<<netLayoutStr<<endl;

                 //process netlayout str - reconfigure net to have this new layout
                //tokenize on |

                int numInpsFile = 0, numHidLyrs = 0, numOuts = 0;
                //second line just has values - not counting bias inputs
                std::getline(netFile,netLayoutStr, '|');
                numInpsFile = stoi(netLayoutStr);     
                std::getline(netFile,netLayoutStr, '|');
                numHidLyrs = stoi(netLayoutStr);
                vector<int> numHidLyrAra(numHidLyrs);
                for(int i = 0; i < numHidLyrs; ++i){
                    std::getline(netFile,netLayoutStr, '|');
                    numHidLyrAra[i] = stoi(netLayoutStr);
                }
                std::getline(netFile,netLayoutStr, '|');
                numOuts = stoi(netLayoutStr);

                    //read past unnecessary data
                std::getline(netFile,valStr);
                std::getline(netFile,valStr);
				int numLayers = hidLayers.size();
				for(int hlyr = 0; hlyr < numLayers; ++hlyr){
					for(int pIdx = 0; pIdx < hidLayers[hlyr].size(); ++pIdx){
                        std::getline(netFile,valStr);                           //kill initial read for hidden layer
                        vector<double> wVals(hidLayers[hlyr][pIdx].inSize);
                        while(std::getline(netFile,valStr,'|')){
                            wVals.push_back(std::stod(valStr));
                        }

                        for(int wIdx = 0; wIdx < hidLayers[hlyr][pIdx].inSize; ++wIdx){
    						hidLayers[hlyr][pIdx].inWts[wIdx] = wVals[wIdx];
                        }
					}//for all perceps
				}//for all hidden layers
                std::getline(netFile,valStr);
				for(int pIdx = 0; pIdx < outLayer.size(); ++pIdx){
                    std::getline(netFile,valStr);                           //kill initial read for hidden layer
                    vector<double> wVals(outLayer[pIdx].inSize);
                    while(std::getline(netFile,valStr,'|')){
                        wVals.push_back(std::stod(valStr));
                    }
		            for(int wIdx = 0; wIdx < outLayer[pIdx].inWts.size(); ++wIdx){
    				    outLayer[pIdx].inWts[wIdx] = wVals[wIdx];
                    }
				}//for all perceps
                netFile.close();
            }//loadNet


    }//namespace kinect
}//namespace rtql8

