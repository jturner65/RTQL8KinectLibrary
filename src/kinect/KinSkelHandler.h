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

#ifndef __KINSKELHANDLER_H__
#define __KINSKELHANDLER_H__
/**
   author : john turner
   class to handle kinect interaction with IK/char controller program - skeleton handling
   requires Microsoft SDK 1.7
**/

#include "KinStreamHandler.h"

using namespace std;

//all functionality to process skeleton data from kinect
//
//avatar refers to graphical character representation of skeleton from cleaned/processed kinect data
//skeleton refers to structure built from actual kinect joint data
//we want an accurate and consistent representation of data from kinect, so we want to process skeleton 
//	data to make a pretty avatar that reflects the user's actions accurately and consistently

namespace rtql8 {
    namespace kinect{
        class KinectHandler;
        class KinStreamHandler;

        class KinSkelHandler : public KinStreamHandler{
        public:
            KinSkelHandler(INuiSensor* _oKin, KinectHandler* _owningKH);
            ~KinSkelHandler(void){}

            //initialization and verification methods
            HRESULT initKinStreamHandler();
            HRESULT initKinDataStream();

            void initHndlrDataStructs();
            bool kinValidSkelAvail();

            //public build methods
            void buildSkeletonData();																			//primary method to build skeleton - grabs skel frame, handles building joints
            void buildFrameFromSkelAvatar();
            void dispBuildSkelMsg(int skelId, int trState);

            //getter setters
            int getCtlSkel(){return ctlSkel;}
            bool hasValidCtlSkel(){return (ctlSkel != -1);}
            vector<double> getClippingBox();
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getBoneOrientVecs();
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getLastKnownSkelJointLocs();
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getKinSkelNoisyMarkers();
            unsigned long getKHSMFlagMode(int idx, bool trackOrFrame);
            bool validCtlSkel() {	return ((0 <= ctlSkel) && (NUI_NumSkels > ctlSkel));}					//returns whether or not there is a valid control skeleton

			vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> getJointVels();
			vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> getJointAccels();
			deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getJointVelsForJoint(int jntIdx);
			deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getJointAccelsForJoint(int jntIdx);	

                //locations purely from kinect, before modification by scale/translation in buildSkelJointLocation
			vector<Vector4> getCurrRawSkelJointLocs();
				//raw skel locations mapped to depth space for both hands - idx's 0,1,2 for left hand, 3,4,5 for right
			Eigen::VectorXi getSkelRawHandDepthPos();
				//raw skel locations  for both hands - idx's 0,1,2 for left hand, 3,4,5 for right
			Eigen::VectorXd getSkelRawHandPos();



            //to-string - type methods
            vector<string> getJointAngleStrs();
            vector<string> getAvtrBoneLenVecStrs();
            vector<string> getBestKinSkelJointLocsStrs();
            vector<int>	getCtlSkelJointState();
            virtual bool setFlag(int idx, bool val);

            bool isLowerLegJnt(int jIdx){return ((jIdx == jIdx_KneeR) || (jIdx == jIdx_KneeL) || (jIdx == jIdx_AnkleR) || (jIdx == jIdx_AnkleL) || (jIdx == jIdx_FootR) ||	(jIdx == jIdx_FootL)) ;}

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        private :	//methods
            void initJointData();
            void initSkelData();
            void initVars();

            //joint functions
            void buildJointLocationArray(bool firstPass, int skelIdx, const NUI_SKELETON_FRAME &skelFr);								//build ara of joints from kinect skeleton tracking - called every skel frame capture
            Eigen::Vector3d buildSkelJointLocation(Vector4 skeletonPoint, int idx);														//converts actual skel frame data to screen/world location - skeletonpoint contains x,y,z,1 where x,y,z are dist from sensor in meters			
            void buildKinSkelJointAngles(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& jointLocs);			//build vectors of local and global joint angles for entire skeleton based on passed joint locs - of dubious utility due to kinect noisiness
            //bone vector funcs
            void buildBoneVectorArray(int skelIdx, bool avtr);
            void buildSnglBoneVector(int skelIdx, int idxJoint, bool goodJoints, bool avtr);

            //set "good" kin joint value based on status of data from kinect
            void setTrackedJointVals(int skelIdx, int jntIdx);
            void setInterpJointVals(int skelIdx, int jntIdx);
            void setNotTrackedJointVals(int skelIdx, int jntIdx);

            //helpers for joint deques data
            void popJntDeqs(int jntIdx);
            void pushJntDeqs(int jntIdx, const Eigen::Vector3d& newPos, int trackedState);
            void findJointDispVals(int jntIdx, double disp);

            //root/leg cleanup functions
            void handleCrouch(bool firstPass, int skelIdx);
            bool checkRootMoved(const Eigen::Vector3d& curRtLoc);														//returns whether or not root has moved beyond some threshold in x-z plane (to recalculate root location
            void cndLegJoints(int skelIdx, double hDiff);																//repairs erroneous leg orientations and enables accurate, reliable squatting
            void resizeKinSkel(int skelIdx);																			//resizes kinect skeleton bone length while maintaining general orientation of joints  using bone lengths from avatar 
            double calcKneeSprdAngle(int skelIdx);																		//calculates the angle between the legs of the passed skeleton
            void calcInferredFloor(const Eigen::Vector4d& kinSkFlr);
            double calcFloorY(double xVal, double zVal);																//calculate the y value on the floor for given x and z values
            double calcDistToFloor(const Eigen::Vector3d& pnt);															//calculate the distance to the floor plane from a point
            int boneQuality(int bnIdx);																					//gets quality of bone based on tracked state of joints(before any modification from crouching)
            void setClipVals(int skelIdx, int jnt);																		//builds arrays of extremal values to display clipping box around kin raw skel data
            void dispClipVals();																						//displays clipping values to console - debug only

            //joint helpers
            bool jntTrk(int state);
            bool jntInf(int state);										
            bool jntUnk(int state);																						//NOTE : difference between NOT_TRACKED as per NUI and !TRACKED (can be INFERRED also)
            bool bothJointsUnknown(int joint0State, int joint1State);													//specifically NOT TRACKED as per definition in NUI
            bool eitherJointNotTracked(int joint0State, int joint1State);												//either joint is inferred or not tracked
            bool bothJointsTracked(int joint0State, int joint1State);
            bool eitherJointInferred(int joint0State, int joint1State);

            HRESULT setStreamEnableFlags();

        private:	//variables
            int msgDisplayCountdown;																			//countdown for next skel msg display
            int curMsgDisplayed;																				//0 for no skel, 1 for skel tracked, 2 for skel pos tracked only
            int ctlSkel;																						//idx of controlling skeleton - should use skeleton with most accurately tracked location markers
			
            Eigen::Vector3d cSkelRootLoc;																		//base root location of currently tracked skeleton - should be location when standing, use to determine crouch.  set when player moves beyond threshold, so assuming upright when moving
            double cSkelRtFlrDist;																				//distance from curr root to current floor plane - updated with every floor update
            Eigen::Vector3d cSkelForward;																		//current skel's forward vector - built from avatar's root/rhip and root/lhip xprod, projected onto x-z plane
            Eigen::Vector4d curFlrPlane;																		//inferred floor plane from skel data - kinect's best guess - need to smooth data
            int cntAtCurFlrEq;																					//number of iterations at this floor eq - max at 30, subtract for each different depth, to build interpolant - when 0, set to new size
            double curFlrEq;																					//a*1000 + b*100 + c*10 + d of current kinect inferred floor - used to determine when to change floor if kinect gets noisy

            //floor clipping values
            double MaxSkelClip[3][3];																			//coords for maximum and minimum values of x,y,z that the kinect will track a skeleton - first idx is extremal coord, 2nd idx is world coord for extremal
            double MinSkelClip[3][3];																			//maybe use skel floor vector from skel structure for floor and ceiling - ignore y value, use floor instead
            double FloorCrnrsY[4];																				//y values of floor at 4 extremal x and z values - 0 minXmaxZ, 1 minXminZ, 2 maxXminZ, 3 maxXmaxZ

            //avatar : clean graphical representation of avatar skeleton, matching 
            //joint location data from kinect
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> avtrOrigJntFrame;					//matches markers for coach avatar, used to prevent illegal/irrational coach positions from ik'ing bad kinect data
            vector<double> avtrOrigBoneSize;																	//the length of each avatar bone - should not change, so only needs to be set once
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> avtrOrigBoneOrientVecs;			//array of normalized original bone orientation vectors for AT REST avatar (graphical rep of skel)-calculated here.
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> avtrOrigRelDispFromRoot;			//array of displacements from root for original avatar skel. - useful for setting location of untracked joints in tracked skeletons
            vector<string> avtrJntFrameNames;																	//string names of each joint

            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> avtrJntFrame;						//matches markers for coach avatar, used to prevent illegal/irrational coach positions from ik'ing bad kinect data
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> avtrBoneOrientVecs;				//array of unit-len bone orientation vectors for avatar (graphical rep of skel)-calculated here.
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> avtrBoneAdjOrientVecs;			//array of actual length bone orientation vectors for avatar (graphical rep of skel)-calculated here.

            //joint values from previous joint positions - 20-vecara (perjoint)									 
            //of jntPosAraDpth-deqara(per iter) of 3-element vecs - all presumed to be for current cltskel
            vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> jntAccels;					//"accels" : changes in displacements previous joints
            vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> jntVels;					//"velocities" : displacements from previous joint pos 
            vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> jntPos;					//previous positions of joints		
            vector<deque<int>> jntPosTracked;																	//previous positions of joint was tracked(2), inferred(1) or not tracked (0)	

            vector<int> jntInterpCount;																			//per joint number of times we have interpolated an inferred/untracked joint - use to weight between interpolation and just taking last avatar value for joint 
            //values used to analyze joint displacements 
            ////- possibly use to cap per-round displacement to help filter joints
            //static const int jntDispMaxIter = 300;															//number of iterations to measure displacement magnitude for each joint for valid joint mapping
            //vector<int> jntDispAccumIter;																		//number of times this process has occured
            //vector<int>	jntDispIter;																		//per-joint count of valid joint displacements - when this hits jntDispMaxIter, take average of sum
            //vector<double> jntDispValidSum;																	//per joint sum of displacements of valid joint mappings	- divide by jntDispMaxIter to get average once that many have been collected	
            //vector<double> jntDispValidMax;																	//per joint max of displacements of valid joint mappings	- collect jntDispMaxIter of these and then store in vec-deq below, corresponding to each avg
            //vector<deque<double>> jntDispAvgAra;																//array of average displacements - to be displayed  
            //vector<deque<double>> jntDispMaxAra;																//array of max displacements per each set of avg displacements shown above - to be displayed  
            //kin skeleton values - per skeleton(currently up to 6) per joint values
            vector<vector<Vector4>> currRawSkelJointLocs;														//array of most recent noisy skeleton joint locations - completely unfiltered
            vector<vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> currBaseSkelJointLocs;	//array of most recent noisy skeleton joint locations - modified by scale/offset but not filtered
            vector<vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> lastKnownSkelJointLocs;	//array of most recent valid skeleton joint locations
            vector<vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> kinSkelJointLocs;			//array of skeletons' joints - up to 6 skels possible, each has up to 20 joints (NUI_SKELETON_POSITION_COUNT = 20 as of 1.7)
            //	NOTE:when first determined from skel data, these are flipped idx-wise to match avatar orientation (left->right)
            vector<int> kinSkelState;																			//per skel ara of most recent skeleton tracking state
            vector<vector<int>>	kinSkelJointState;																//per skel ara of most recent per joint tracking state for filtered kin skel - purely determined by kinect engine (this avoids the need to retain reference to skel frame)

            //bones here are kinect "bones" - vectors between adjacent skeletons in kin skel
            vector<vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> boneOrientVecs;			//array of unit-len bone orientation vectors based on joint locations
            vector<vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> boneAdjOrientVecs;		//adjusted array of bone orientation vectors, set to length based on avatar skel bone length
            vector<vector<int>>	 boneTrackedState;																//state of whether bone is tracked - if both joints tracked, bone tracked(2), if either inferred, bone inferred(1), if either not tracked, bone not tracked(0)

            vector<vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>> skelJointAnglesGlobal;	//20 joints -> 20 angles in global coords - axis angle, for each skeleton

            vector<int> skelsTracked;																			//idx's of skeletons currently tracked
            vector<int> skelsRootOnlyTracked;																	//idx's of skeletons with only position of root data available
        };
    }//namespace kinect
}//namespace rtql8 
#endif