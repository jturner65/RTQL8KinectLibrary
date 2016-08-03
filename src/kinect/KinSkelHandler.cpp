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
#include "KinSkelHandler.h"

using namespace std;

namespace rtql8{
    namespace kinect{

        //NUI_NumJnts -> num kinect joints, will change with next kinect hardware/sdk release
        //NUI_NumSkels -> num total possible skels tracked.  will also change
        //NOTE : kinect skeleton is mirrored of user, so left knee joint of kinect skel (with respect to skel itself) is actually user's right knee joint

        KinSkelHandler::KinSkelHandler(INuiSensor* _oKin, KinectHandler* _owningKH) : KinStreamHandler(_oKin, _owningKH, _H_SKEL),
                                                                                      msgDisplayCountdown(0), curMsgDisplayed(-1), ctlSkel(-1), cSkelRootLoc(0,0,0), cSkelRtFlrDist(0), cSkelForward(0,0,0), curFlrPlane(0,0,0,0), cntAtCurFlrEq(0), curFlrEq(0),
                                                                                      avtrOrigJntFrame(NUI_NumJnts,Eigen::Vector3d(0, 0, 0)), avtrOrigBoneSize(NUI_NumBones), avtrOrigBoneOrientVecs(NUI_NumBones,Eigen::Vector3d(0, 0, 0)), 
            avtrOrigRelDispFromRoot(NUI_NumJnts,Eigen::Vector3d(0, 0, 0)), avtrJntFrameNames(NUI_NumJnts), avtrJntFrame(NUI_NumJnts,Eigen::Vector3d(0, 0, 0)), avtrBoneOrientVecs(NUI_NumBones,Eigen::Vector3d(0, 0, 0)), 
            avtrBoneAdjOrientVecs(NUI_NumBones,Eigen::Vector3d(0, 0, 0)), jntAccels(NUI_NumJnts), jntVels(NUI_NumJnts), jntPos(NUI_NumJnts), jntPosTracked(NUI_NumJnts), jntInterpCount(NUI_NumJnts,0),
        //used to determine range of valid joint displacements per unit time, not necessary for running handler
        //jntDispAccumIter(NUI_NumJnts), jntDispIter(NUI_NumJnts), jntDispValidSum(NUI_NumJnts), jntDispValidMax(NUI_NumJnts), jntDispAvgAra(NUI_NumJnts), jntDispMaxAra(NUI_NumJnts),	
            currRawSkelJointLocs(NUI_NumSkels), currBaseSkelJointLocs(NUI_NumSkels), lastKnownSkelJointLocs(NUI_NumSkels), kinSkelJointLocs(NUI_NumSkels), kinSkelState(NUI_NumSkels), kinSkelJointState(NUI_NumSkels), 
            boneOrientVecs(NUI_NumBones), boneAdjOrientVecs(NUI_NumBones), boneTrackedState(NUI_NumBones), skelJointAnglesGlobal(NUI_NumSkels), skelsTracked(NUI_NumSkels),skelsRootOnlyTracked(NUI_NumSkels) 
        {

            HRESULT tmpRes = initKinStreamHandler();//call local
        }//cnstrctr

        HRESULT KinSkelHandler::initKinStreamHandler(){
            clog<<"\tKinSkelHandler Log : call base from skel init class"<<std::endl;
            HRESULT tmpRes = KinStreamHandler::initKinStreamHandler();		//base class
            clog<<"\tKinSkelHandler Log : call skel data structs init"<<std::endl;
            initHndlrDataStructs();
            return tmpRes;
        }//skel specific init streams

        //initialize structures - NUI_NumJnts = 20, but will increase with next kinect
        void KinSkelHandler::initHndlrDataStructs(){							//must only be called from initKinStreamHandler
            initVars();
            initJointData();
            initSkelData();
        }//initSkelHndlrDataStructs

        void KinSkelHandler::initJointData(){
            //jntDispAccumIter.clear();								//used to calculate reasonable displacements for skeleton joints
            //jntDispIter.clear();		
            //jntDispValidSum.clear();		
            //jntDispValidMax.clear();		

            for (int idx = 0; idx < NUI_NumJnts; ++idx){						//depth of aras to hold stencil values of kin skeleton
                jntAccels[idx].clear();	
                jntVels[idx].clear();
                jntPos[idx].clear();
                //jntDispAvgAra[idx].clear();									//array of average displacements - to be displayed  
                //jntDispMaxAra[idx].clear();									//array of max displacements per each set of avg displacements shown above - to be displayed  
                jntPosTracked[idx].clear();
                for(int dIdx = 0; dIdx < jntPosAraDpth; ++dIdx){
                    jntAccels[idx].push_front(vec3_zero);	
                    jntVels[idx].push_front(vec3_zero);	
                    jntPos[idx].push_front(vec3_zero);
                    //jntDispAvgAra[idx].push_front(0);
                    //jntDispMaxAra[idx].push_front(0);
                    jntPosTracked[idx].push_front(0);
                }//for each deque pos
            }//for each jntposara element
        }//initJointData

        void KinSkelHandler::initSkelData(){
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> tmpVec(NUI_NumJnts, vec3_zero);
            vector<Vector4> tmpVector4(NUI_NumJnts);
            vector<int> tmpVec2(NUI_NumJnts);
            vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> tmpVec4(NUI_NumJnts, vec4_zero);

            vector<int> bonetmpVec(NUI_NumBones);
            vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> boneTmpVec2(NUI_NumBones, vec3_zero);

            avtrJntFrame = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(boneTmpVec2);
            avtrBoneOrientVecs = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(boneTmpVec2);					// for each skel num bones		
            avtrBoneAdjOrientVecs = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(boneTmpVec2);					// for each skel num bones		

            for (int idx = 0; idx < NUI_NumSkels; ++idx){			
                boneOrientVecs[idx] = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(boneTmpVec2);				//6 skels, orientation of kinect "bones" - vectors between joints - for all skeletons
                boneAdjOrientVecs[idx] = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(boneTmpVec2);			//6 skels, orientation of kinect "bones" - adj vectors between joints based on avatar skel - for all skeletons
                boneTrackedState[idx] = vector<int>(bonetmpVec);

                kinSkelJointLocs[idx] = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(tmpVec);					//this is data directly from kinect, including inferred joints
                kinSkelState[idx] = 0;																								//states, inited to 0
                kinSkelJointState[idx] = vector<int>(tmpVec2);																		//20 possible joint states, inited to 0
                currRawSkelJointLocs[idx] = vector<Vector4>(tmpVector4);															//current ctlSkel's idx holds unfiltered skeleton joint data
                currBaseSkelJointLocs[idx] = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(tmpVec);				//current ctlSkel's idx holds unfiltered skeleton joint data
                lastKnownSkelJointLocs[idx] = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(tmpVec);			//this is intended to be clean/legitimate joint location data

                skelJointAnglesGlobal[idx] =  vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>(tmpVec4);			//20 joints -> 20 angles in global/worldspace coords - axis angle
            }//for each skeleton
        }//initSkelData()

        void KinSkelHandler::initVars(){
            STRM_Flags = vector<bool>(SK_numFlags, false);              //TODO: investigate this
            msgDisplayCountdown = 0;
            curMsgDisplayed = -1;
            ctlSkel = -1;
            cSkelRootLoc = (Eigen::Vector3d(0, 0, 0));
            cSkelForward = (Eigen::Vector3d(0, 0, 0));
            curFlrPlane = (Eigen::Vector4d(0, 0, 0, 0));
            cSkelRtFlrDist = 0;
            cntAtCurFlrEq = 0;
            curFlrEq = 0;
            for(int i=0;i<3;++i){for(int j=0;j<3;++j){	MaxSkelClip[i][j] = -1;	MinSkelClip[i][j] = 1;	}}
            for(int i=0;i<4;++i){FloorCrnrsY[i] = 1;}
            //preserved flags data
            bool tmpAvSetData = STRM_Flags[_SK_AVTR_INIT_SET];               //TODO: investigate this
            for(int i =0; i < SK_numFlags; ++i){STRM_Flags[i] = false;}
            STRM_Flags[_SK_AVTR_INIT_SET] = tmpAvSetData;								//preserve avatar data being set
        }//initVars

        HRESULT KinSkelHandler::initKinDataStream(){ 
            if(!HasSkeletalEngine(owningKinect)){		return ((HRESULT)-1l);	}		//negative value is failure
            //first arg is for event
            owningKH->setFlag(_SK_NEAR_IDX,true);
            HRESULT skeStrmOpenRes = setStreamEnableFlags();
            return skeStrmOpenRes;
        }//initSkelStream

        //sets/resets skeleton tracking with appropriate flags for near and seated support based on state machine - using the event-driven model
        HRESULT KinSkelHandler::setStreamEnableFlags(){
            HRESULT skelStrmOpenRes = owningKinect->NuiSkeletonTrackingEnable(m_hNextEvent,						//set whenever new frame is ready, reset whenever current frame is returned
                                                                              (owningKH->getFlag(_KH_SEATED_IDX) ? NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT : 0) | 
                                                                              (owningKH->getFlag(_KH_NEAR_IDX) ? NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE : 0) |
                                                                              NUI_SKELETON_TRACKING_FLAG_SUPPRESS_NO_FRAME_DATA	);											//suppresses "no frame data" error			
            return skelStrmOpenRes;
        }//setStreamEnableFlags

        bool KinSkelHandler::kinValidSkelAvail(){ return (owningKinect != NULL) && ((owningKH->getFlag(_KH_SKEL_TRACKED_IDX)) || (owningKH->getFlag(_KH_SKEL_ROOTONLY_IDX)));}

        //builds skeleton frame representing actual graphical avatar, to override dirty kinect data avatarBoneOrientVecs
        void KinSkelHandler::buildFrameFromSkelAvatar(){
            avtrJntFrame = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(owningKH->_tmpSkelIKHandles);																//can be idx'ed via NUI_SKELETON_POSITION_INDEX enum vals
            for(int i = 0; i < NUI_NumSkels; ++i){ 
                lastKnownSkelJointLocs[i] = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(owningKH->_tmpSkelIKHandles);											//updating last known good skel joint locs to be joint locs from avatar
            }		//set all skeletons to have avatar's handles locations as baseline "legit" locations
            if(!STRM_Flags[_SK_AVTR_INIT_SET]){//first call - after skeleton is first built successfully and avatar is IK'ed to these will not change
                avtrOrigJntFrame = vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(owningKH->_tmpSkelIKHandles);	
                avtrJntFrameNames = vector<string>(owningKH->_tmpSkelIKHandleNames);													//set names of handles
            }
        }//buildFrameFromSkelAvatar

        //display current skel tracking status
        void KinSkelHandler::dispBuildSkelMsg(int skelId, int trState){
            if((msgDisplayCountdown <= 0) || (curMsgDisplayed != trState)){
                curMsgDisplayed = trState;
                msgDisplayCountdown = SK_MSG_DISP;
                if(NUI_SKELETON_NOT_TRACKED == trState){	clog<<"\tKinSkelHandler Log : No skeleton tracked"<<std::endl; }
                else if(NUI_SKELETON_TRACKED == trState){	clog<<"\tKinSkelHandler Log : Skeleton tracked id : "<<skelId<<std::endl;}
                //else {										clog<<"KinSkelHandler Log : Skeleton center only tracked id : "<<skelId<<std::endl;}
            }
            msgDisplayCountdown--;
        }//dispBuildSkelMsg

        //gets skeleton data from kinect to set appropriate values for display avatar
        //hip center marker is root of kinect skeleton data
        void KinSkelHandler::buildSkeletonData(){
            if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextEvent, 0)){															//checking event
                NUI_SKELETON_FRAME skeletonFrame = {0};																			//need to pass this to interactions
                bool firstPass = false;
                skelsTracked.clear();		
                skelsRootOnlyTracked.clear();
                owningKH->clearSkelFlags();

                if (! SUCCEEDED( owningKinect->NuiSkeletonGetNextFrame(0, &skeletonFrame))) {	return; }						//wait for 0 ms, use event that frame is ready to trigger frame read
                // smooth out the skeleton data
                owningKinect->NuiTransformSmooth(&skeletonFrame, &skelSmoothing);												//smoothing data is struct NUI_TRANSFORM_SMOOTH_PARAMETERS 
                //attempt to bypass checking entire array every frame, to speed up processing
                if((-1 != ctlSkel) && (NUI_SKELETON_TRACKED == skeletonFrame.SkeletonData[ctlSkel].eTrackingState)){            //tracked skeleton last frame, tracking same one this frame too - not firstpass
                    dispBuildSkelMsg(ctlSkel, NUI_SKELETON_TRACKED);
                    kinSkelJointState[ctlSkel] = vector<int>(NUI_NumJnts);													    //each joint will have tracking value of 0,1,or2, set in buildJointLoc method below
                    owningKH->setFlag(_KH_SKEL_TRACKED_IDX, true);				
                    skelsTracked.push_back(ctlSkel);
                    kinSkelState[ctlSkel] = 2;																					//skel tracking is 2
                    buildJointLocationArray(firstPass, ctlSkel, skeletonFrame);
                } else {                                                                                                        //look for new skeleton
                    NUI_SKELETON_TRACKING_STATE trackingState;
                    ctlSkel = -1;
                    //	//send skel frame data to interaction handler
                    //owningKH->ProcessSkeleton(skeletonFrame);
                    //need to handle multiple skeletons - up to 6 partial skeletons can be tracked (roots only) with up to 2 of them being full skeletons
                    for (int i = 0 ; i < NUI_NumSkels; ++i){//for each possible skeleton...
                        trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
                        kinSkelJointState[i] = vector<int>(NUI_NumJnts);															//each joint will have tracking value of 0,1,or2, set in buildJointLoc method below
                        if (NUI_SKELETON_TRACKED == trackingState){																	// We're tracking the skeleton at this branch, process its data - 20 joints tracked in skeleton 
                            dispBuildSkelMsg(i, trackingState);
                            owningKH->setFlag(_KH_SKEL_TRACKED_IDX, true);				
                            skelsTracked.push_back(i);
                            if(ctlSkel == -1){		
                                //clog<<"ctl skel <--- -1 : "<<ctlSkel<<" i :"<<i<<std::endl;
                                //ctlSkel = i;	
                                firstPass = true;
                            }//set the controlling skeleton to be this one, if there isn't one already - need to more accurately determine which tracked skel is cntl skel
                            ctlSkel = i;	
                            //else if (ctlSkel != i){			//skeleton seems to jump idx's - changes from one idx to another without being lost by kinect
                            //clog<<"ctl skel != i, -1 : "<<ctlSkel<<" i :"<<i<<std::endl;
                            //}
                            // TODO : if this is the currently tracked skel, set camera tilt to be focused on skel root if not set already - need to then account for new ground plane when rendering skel(i.e. rotate points around root before IK)
                            kinSkelState[i] = 2;																					//skel tracking is 2
                            buildJointLocationArray(firstPass, i, skeletonFrame);
                        }
                        else if (NUI_SKELETON_POSITION_ONLY == trackingState){														// we've only received the center point of the skeleton at this branch, use this as root(hip)
                            if(!owningKH->getFlag(_KH_SKEL_TRACKED_IDX)) {owningKH->setFlag(_KH_SKEL_ROOTONLY_IDX, true);}
                            dispBuildSkelMsg(i, trackingState);
                            skelsRootOnlyTracked.push_back(i);
                            kinSkelState[i] = 1;																//root only tracking is 1
                            kinSkelJointState[i][0] = 1;														//this skel only has root tracking, so idx 0 is 1 (inferred)
                        } else {//this skeleton id not tracked
                            kinSkelState[i] = 0;																//untracked is 0
                        }
                    }//for each skeleton
                }//if no ctl skel found
                //check if any skeletons are currently tracked or at least recognized positionally
                if((skelsTracked.size() == 0) && (skelsRootOnlyTracked.size() == 0)){//no skeletons found,
                    dispBuildSkelMsg(-1, NUI_SKELETON_NOT_TRACKED);
                    owningKH->setFlag(_KH_SKEL_NONEFOUND_IDX, true);
                    STRM_Flags[_SK_CAM_TILT_SET] = false;
                    if(ctlSkel != -1) {																		//had a tracked skeleton, but now it has been lost - clear out all skel data
                        clog<<"ctl skel --> -1 : "<<ctlSkel<<std::endl;

                        for(int j = 0; j < NUI_NumJnts; ++j){
                            lastKnownSkelJointLocs[ctlSkel][j] = Eigen::Vector3d(avtrOrigJntFrame[j]);		//reset all last known joint locations - intention here is to have the avatar stop in space if the skeleton disappears  
                            avtrJntFrame[j] = Eigen::Vector3d(avtrOrigJntFrame[j]);
                        }																					//avatar should move to neutral/starting position until another skeleton is tracked
                        buildBoneVectorArray(-1, true);	
                        buildBoneVectorArray(ctlSkel, false);												//build arrays of kin skel bone vectors						
                        resizeKinSkel(ctlSkel);																//rebuild skeleton from root outward
                        owningKH->setFlag(_KH_SKH_VLD_JNTL_IDX, true);
                        cSkelRootLoc = Eigen::Vector3d(0,0,0);
                        cSkelForward = Eigen::Vector3d(0,0,0);
                        ctlSkel = -1;														//no ctl skel
                    }//if there had been a ctl skel but now there isn't
                    else {
                        owningKH->setFlag(_KH_SKH_VLD_JNTL_IDX, false);
                    }
                }//no skels found 
                if(ctlSkel != -1){
                    //send skel frame data to interaction handler if skel tracked
                    owningKH->ProcessSkeleton(skeletonFrame);
                }
            }//if event triggered skel frame ready
        }//buildSkeletonData

        //set joint location value for tracked joint
        void KinSkelHandler::setTrackedJointVals(int skelIdx, int jntIdx){
            jntInterpCount[jntIdx] = 0;														//clear interpolation/inferred count for this joint, since we have a good value
            popJntDeqs(jntIdx);																//make room in pos,vel,acc deques
            Eigen::Vector3d tmpVec = Eigen::Vector3d(kinSkelJointLocs[skelIdx][jntIdx]);
            pushJntDeqs(jntIdx, tmpVec, 2);				//set new values for pos,vel,acc deques
        }//setTrackedJointVals
	
        //set joint location val for inferred joint
        void KinSkelHandler::setInterpJointVals(int skelIdx, int jntIdx){
            ++jntInterpCount[jntIdx];																				//increment number of inferred processes - used to pick between cur avatar joint value and interpolated kin skel value
            popJntDeqs(jntIdx);
            if (jntInterpCount[jntIdx] < jntNumMaxInterp){
                double intrp = (jntInterpCount[jntIdx]/(1.0*jntNumMaxInterp));										//interpolant - use num of prev interpolations for this joint
                Eigen::Vector3d interpPos(kinSkelJointLocs[skelIdx][jntIdx]			);
                //change to last good joint position
                //+ intrp * ((avtrOrigRelDispFromRoot[jntIdx] + kinSkelJointLocs[skelIdx][jIdx_ROOT]) - kinSkelJointLocs[skelIdx][jntIdx]));					//position should be most recent avatar joint position + some interpolant toward interpolated position
                pushJntDeqs(jntIdx, interpPos,false);
            } else {	
                Eigen::Vector3d tmpVec = Eigen::Vector3d(kinSkelJointLocs[skelIdx][jIdx_ROOT] + avtrOrigRelDispFromRoot[jntIdx]);
                pushJntDeqs(jntIdx, tmpVec,1);			//current root (Which will be tracked) + original displacement of avatar skel
            }
        }//setInterpJointVals

        //set joint value for non-tracked joint - use values from avatar, since avatar should be matched to recent values of kin skel and should be legal
        void KinSkelHandler::setNotTrackedJointVals(int skelIdx, int jntIdx){
            popJntDeqs(jntIdx);	
            Eigen::Vector3d tmpVec = Eigen::Vector3d(avtrOrigRelDispFromRoot[jntIdx] + kinSkelJointLocs[skelIdx][jIdx_ROOT]);
            pushJntDeqs(jntIdx, tmpVec, 0);				//current root (Which will be tracked) + original displacement of avatar skel
        }//setInterpJointVals

        void KinSkelHandler::popJntDeqs(int jntIdx){		
            jntAccels[jntIdx].pop_back(); 
            jntVels[jntIdx].pop_back(); 
            jntPos[jntIdx].pop_back();
            jntPosTracked[jntIdx].pop_back();
        }//popJntDeqs	

        //calculate current pos,vel, accel for a particular joint based on prev values
        void KinSkelHandler::pushJntDeqs(int jntIdx, const Eigen::Vector3d& newPos, int trackedState){
            Eigen::Vector3d setNewPos(newPos);
            jntPosTracked[jntIdx].push_front(trackedState);														//set this position as tracked or not
            jntPos[jntIdx].push_front(setNewPos);
            Eigen::Vector3d setNewVel = Eigen::Vector3d(jntPos[jntIdx][0] - jntPos[jntIdx][1]);
            if(setNewVel.norm() > jntMaxDisp[jntIdx]){															//don't allow joints to move more than a specific amount per update
                setNewVel.normalize();
                setNewVel *= jntMaxDisp[jntIdx];
                jntPos[jntIdx][0] =  Eigen::Vector3d(jntPos[jntIdx][1] + setNewVel);							//move new joint location back to match displacement
            }																									//scale displacement to not exceed max expected
            jntVels[jntIdx].push_front(setNewVel);																//setting "h" to be 1 - arbitrary
            //give magnitude of displacement off to account structs so that valid displacements can be recorded
            //if((jntPosTracked[jntIdx][0]!=0) && (jntPosTracked[jntIdx][1]!=0)){	findJointDispVals(jntIdx, len(setNewVel));}			
            Eigen::Vector3d setNewAcc = Eigen::Vector3d(jntVels[jntIdx][0] - jntVels[jntIdx][1]);
            jntAccels[jntIdx].push_front(setNewAcc);				
        }//pushJntDeqs

        //returns whether or not root has moved beyond some threshold in x-z plane, or increased in altitude(aka stood up from sitting) using m-hat dist for speed
        //(to recalculate base root location to use for crouching determination)
        bool KinSkelHandler::checkRootMoved(const Eigen::Vector3d& curRtLoc){
            bool movingUp = ((curRtLoc[1] - cSkelRootLoc[1]) > SK_STND_THRESH);														//increasing altitude
            bool movingOut = ((abs(cSkelRootLoc[0] - curRtLoc[0]) + abs(cSkelRootLoc[2] - curRtLoc[2])) > SK_MV_THRESH);			//z and/or x have changed sufficiently to recalc
            return ((!movingUp && movingOut) || (movingUp && !movingOut));
            //if (result) {clog<<" move root loc cur loc : "<<curRtLoc<<"  cSkelRootLoc : "<<cSkelRootLoc<<"   "<<std::endl;}
            //return result;
        }//checkRootMoved

        //calculate the distance to the floor plane from a point using dist = abs(axp + byp + czp + d)/sqrt(a^2 + b^2 + c^2)
        //NOTE : point has been displaced using buildSkelJointLocation, but since all points passed to this should have been, no need to adjust in this 
        //distance measurement so long as distances are only used as comparison - to get actual distance, need to undo displacement of pnt from buildSkelJointLocation
        double KinSkelHandler::calcDistToFloor(const Eigen::Vector3d& pnt){
            double num = 0, denom = 0, retVal = 0;
            for(int i = 0; i < 3; ++i){
                num += curFlrPlane[i] * pnt[i];
                denom += curFlrPlane[i] * curFlrPlane[i];
            }//for each factor
            num += curFlrPlane[3];
            denom = (0 >= denom ? 1 : sqrt(denom));
            retVal = abs(num/denom);
            return retVal;
        }//calcDistToFloor

        //build the inferred floor eq from kinect (4d vec holding a,b,c,d of floor eq ax+by+cz+d=0), with smoothing focused based on changes in d
        void KinSkelHandler::calcInferredFloor(const Eigen::Vector4d& kinSkFlr){
            double tmpFlrEqVals = (1000 * kinSkFlr[0]) +  (100 * kinSkFlr[1]) +  (10 * kinSkFlr[2]) + kinSkFlr[3];					//used to encode values
            //clog<<"tmp flr eq "<<tmpFlrEqVals<< " curFlrEq "<<curFlrEq<<std::endl;
            if(curFlrEq != tmpFlrEqVals){
                --cntAtCurFlrEq;
                if(0 >= cntAtCurFlrEq){
                    curFlrPlane = Eigen::Vector4d(kinSkFlr);																					//kinect's best guess for the floor plane for this skeleton
                    curFlrEq = tmpFlrEqVals;																						//set encoded floor value
                    cntAtCurFlrEq = MX_CNT_FLR_Y;
                }//if we've not been at the current floor location for MX_CNT_FLR_Y iterations, change it
            }//if the current floor location	
        }//buildInferredFloor

        //build ara of joints from kinect skeleton tracking
        void KinSkelHandler::buildJointLocationArray(bool firstPass, int skelIdx, const NUI_SKELETON_FRAME &skelFr){
            const NUI_SKELETON_DATA skel = skelFr.SkeletonData[skelIdx];
            Eigen::Vector4d tmpFloor = Eigen::Vector4d(skelFr.vFloorClipPlane.x, skelFr.vFloorClipPlane.y, skelFr.vFloorClipPlane.z, skelFr.vFloorClipPlane.w);
            calcInferredFloor(tmpFloor);																										//kinect's best guess for the floor plane for this skeleton

            for (int jntIdx=0; jntIdx<NUI_NumJnts;++jntIdx){
                currRawSkelJointLocs[skelIdx][jntIdx] = skel.SkeletonPositions[kinJnts[jntIdx]];
                kinSkelJointLocs[skelIdx][jntIdx] = buildSkelJointLocation(skel.SkeletonPositions[kinJnts[jntIdx]], jntIdx);					//swapping joint idx's to match avatar orientation - use pre-calced root for root
                currBaseSkelJointLocs[skelIdx][jntIdx] = Eigen::Vector3d(kinSkelJointLocs[skelIdx][jntIdx]);									//save noisy kinect skel joints
                if (jntTrk(skel.eSkeletonPositionTrackingState[jntIdx])){																		//if the position is correctly tracked then set the position in the "good joint locs" ara
                    kinSkelJointState[skelIdx][jntIdx] = 2;																						//means this joint is tracked
                    setTrackedJointVals(skelIdx, jntIdx);				
                    setClipVals(skelIdx,jntIdx);																								//if tracked, set min and max pre-clipping plane vals for x,y,z
                } else if (jntInf(skel.eSkeletonPositionTrackingState[jntIdx])){																//if inferred then attempt to place joint at appropriate location and in the right direction based on inferred location 
                    kinSkelJointState[skelIdx][jntIdx] = 1;																						//means this joint is inferred by kinect
                    setInterpJointVals(skelIdx, jntIdx);
                    setClipVals(skelIdx,jntIdx);																								//if inferred, set min and max pre-clipping plane vals for x,y,z
                } else {																														//joint not tracked
                    kinSkelJointState[skelIdx][jntIdx] = 0;																						//means this joint is not tracked
                    setNotTrackedJointVals(skelIdx, jntIdx);
                }			//joint not tracked		
                lastKnownSkelJointLocs[skelIdx][jntIdx] = Eigen::Vector3d(jntPos[jntIdx][0]);																//these are the joints that will be used to build avatar matching skel
            }//for each skeleton position

            //if(STRM_Flags[_SK_INT_CROUCH]){handleCrouch(firstPass, skelIdx);}																	//if crouch interpolation enabled, calculated crouch for inferred/unknown joints from root height change
            buildBoneVectorArray(-1, true);																										//build arrays of avatar bone vector arrangements
            buildBoneVectorArray(skelIdx, false);																								//build arrays of kin skel bone vectors		
            resizeKinSkel(skelIdx);																											//rebuild skeleton from root outward
            owningKH->setFlag(_KH_SKH_VLD_JNTL_IDX, true);
        }//buildJointLocationArray

        //raw skel locations mapped to depth space for both hands, root and rightshoulder - idx's 0,1,2 for left hand, 3,4,5 for right, 6,7,8 for root, 9,10,11 for right shoulder
        //used to manually track hand position for grip determination
        Eigen::VectorXi KinSkelHandler::getSkelRawHandDepthPos(){
            Eigen::VectorXi res(12);
            long lx=0, ly=0, rx=0, ry=0, cx=0, cy=0, sx=0, sy=0;
            USHORT ld=0, rd=0, cd=0, sd=0;			
            //swapping back idx's - depth data has actual hand locations appropriately oriented		
            Vector4 locL(currRawSkelJointLocs[ctlSkel][jIdx_HandR]), locR (currRawSkelJointLocs[ctlSkel][jIdx_HandL]), locC (currRawSkelJointLocs[ctlSkel][jIdx_ROOT]), locS (currRawSkelJointLocs[ctlSkel][jIdx_ShldrL]);	
            //used by depth handler to find hands, use wrist instead of hand joint
            //Vector4 locL(currRawSkelJointLocs[ctlSkel][jIdx_WristR]), locR (currRawSkelJointLocs[ctlSkel][jIdx_WristL]), locC (currRawSkelJointLocs[ctlSkel][jIdx_ROOT]), locS (currRawSkelJointLocs[ctlSkel][jIdx_ShldrL]);						
            NuiTransformSkeletonToDepthImage(locL, &lx, &ly, &ld, dpthInitRes);
            NuiTransformSkeletonToDepthImage(locR, &rx, &ry, &rd, dpthInitRes);
            NuiTransformSkeletonToDepthImage(locC, &cx, &cy, &cd, dpthInitRes);
            NuiTransformSkeletonToDepthImage(locS, &sx, &sy, &sd, dpthInitRes);
            res<<(int)lx,(int)ly,(int)(ld>>3),(int)rx,(int)ry,(int)(rd>>3),(int)cx,(int)cy,(int)(cd>>3),(int)sx,(int)sy,(int)(sd>>3);	//need to get rid of rightmost 3 bits for depth
            return res;
        }//getSkelRawHandDepthPos

			//raw skel locations for both hands, root and rightshoulder - idx's 0,1,2 for left wrist, 3,4,5 for right, 6,7,8 for root, 9,10,11 for right shoulder
		Eigen::VectorXd KinSkelHandler::getSkelRawHandPos(){
			Eigen::VectorXd res(12);
			res<<currRawSkelJointLocs[ctlSkel][jIdx_WristR].x 
                ,currRawSkelJointLocs[ctlSkel][jIdx_WristR].y 
                ,currRawSkelJointLocs[ctlSkel][jIdx_WristR].z
                ,currRawSkelJointLocs[ctlSkel][jIdx_WristL].x 
                ,currRawSkelJointLocs[ctlSkel][jIdx_WristL].y 
                ,currRawSkelJointLocs[ctlSkel][jIdx_WristL].z
                ,currRawSkelJointLocs[ctlSkel][jIdx_ROOT].x 
                ,currRawSkelJointLocs[ctlSkel][jIdx_ROOT].y 
                ,currRawSkelJointLocs[ctlSkel][jIdx_ROOT].z
                ,currRawSkelJointLocs[ctlSkel][jIdx_ShldrL].x 
                ,currRawSkelJointLocs[ctlSkel][jIdx_ShldrL].y
                ,currRawSkelJointLocs[ctlSkel][jIdx_ShldrL].z;
			return res;
		}//getSkelRawHandDepthPos

		//handle guessing crouch data if enabled
		void KinSkelHandler::handleCrouch(bool firstPass, int skelIdx){
			clog<<"\tKinSkelHandler Log : handle crouch interpolation/extrapolation for skel id : "<<skelIdx<<std::endl;
			Eigen::Vector3d rootLoc = lastKnownSkelJointLocs[skelIdx][jIdx_ROOT];																			//tmp root loc for this iteration

            if((firstPass) || (checkRootMoved(rootLoc))){																						//reset standing root location if player moves more than some threshold, up above previous setting, or if first time through with skeleton
                cSkelRootLoc = rootLoc;																											//need to reset from lateral movement because kin might be tilted or not perfectly aimed at root, giving different positions different heights when standing
                cSkelRtFlrDist = calcDistToFloor(cSkelRootLoc);																					//distance to floor from current tracked skeleton's root when standing
                clog<<"\tKinSkelHandler Log : init standing root set : "<<cSkelRootLoc<<" dist to floor : "<<cSkelRtFlrDist<<std::endl;
            }

            //double curRtDist = calcDistToFloor(rootLoc) ,																						//current distance from root to floor
            //	curHeightDiff = cSkelRtFlrDist - curRtDist;																						//difference between recorded height diff to floor and cur dist to floor : >0 for crouching

            //if(curHeightDiff > 0){
            //	//clog<<"---crch    curSkel y : "<<lastKnownSkelJointLocs[skelIdx][jIdx_ROOT][1]<<" cur rt hght : "<<curRtDist<<" cSkelrt y : "<<cSkelRootLoc[1]<<" dist2flr : "<<cSkelRtFlrDist<<"  hdiff : "<<curHeightDiff<<"    "<<std::endl; 			
            //	if ((2 != kinSkelJointState[skelIdx][jIdx_KneeL]) && (2 != kinSkelJointState[skelIdx][jIdx_KneeR])){							//neither knee tracked																												//crouching/lowering center of mass - need to be called after bone vectors are built
            //		cndLegJoints(skelIdx, curHeightDiff);																						//move feet and make crouch layout if determined to be correct
            //	} else if (!((2 == kinSkelJointState[skelIdx][jIdx_KneeL]) && (2 == kinSkelJointState[skelIdx][jIdx_KneeR]))){					//only 1 knee tracked - set height of untracked knee to be that of tracked knee
            //		if (2 == kinSkelJointState[skelIdx][jIdx_KneeL]){	lastKnownSkelJointLocs[skelIdx][jIdx_KneeR][1] = lastKnownSkelJointLocs[skelIdx][jIdx_KneeL][1];}
            //		else {												lastKnownSkelJointLocs[skelIdx][jIdx_KneeL][1] = lastKnownSkelJointLocs[skelIdx][jIdx_KneeR][1];}
            //	}//if neither knee tracked else if 1 but not the other tracked
            //}//if root lower than base root of ctl skel means crouching

        }//handleCrouch

        //build bone orientation vectors of current ctl skel, if there is one
        void KinSkelHandler::buildBoneVectorArray(int skelIdx, bool isAvtr){
            for(int bnIdx = 0; bnIdx < NUI_NumBones; ++bnIdx){			
                if(isAvtr) {																										//if avatar, store results from previous iteration of IK, to be used to reconstruct skeleton from kin data
                    Eigen::Vector3d jntA(avtrJntFrame[bnJoints[bnIdx][0]]), jntB(avtrJntFrame[bnJoints[bnIdx][1]]), tmpVec;
                    tmpVec = ((jntA == jntB) ? vec3_zero : (Eigen::Vector3d(jntA - jntB)));
                    avtrBoneAdjOrientVecs[bnIdx] = tmpVec;																			//adj orient vecs are actual len
                    avtrBoneOrientVecs[bnIdx] = tmpVec.normalized();																//non-adj orient vecs are normalized
                    if (!STRM_Flags[_SK_AVTR_INIT_SET])	{																			//if avatar bones haven't been set yet - 1st time through for avatar
                        avtrOrigBoneSize[bnIdx] = tmpVec.norm();
                        clog<<"\tKinSkelHandler Log : Avatarbone size["<<bnIdx<<"] = "<<avtrOrigBoneSize[bnIdx]<<std::endl;
                        avtrOrigBoneOrientVecs[bnIdx] = Eigen::Vector3d(avtrBoneOrientVecs[bnIdx]);
                    }
                } else {																											//if bones of kin skel data, get calculated joint locations and use to determine orientation vectors
                    int boneQual = boneQuality(bnIdx); 																			
                    boneTrackedState[skelIdx][bnIdx] = boneQual;																	//tracking state of bone
                    if(0 != boneQual){
                        Eigen::Vector3d jntA(lastKnownSkelJointLocs[skelIdx][bnJoints[bnIdx][0]]), jntB(lastKnownSkelJointLocs[skelIdx][bnJoints[bnIdx][1]]);
                        boneOrientVecs[skelIdx][bnIdx] = ((jntA == jntB) ? vec3_zero : Eigen::Vector3d(jntA - jntB)); 
                        boneAdjOrientVecs[skelIdx][bnIdx] = (boneOrientVecs[skelIdx][bnIdx].normalized()) * avtrOrigBoneSize[bnIdx];	//set these vectors to be as long as avatar bones, if avatar bone length has been set, otherwise keep actual length
                    } else {//if joints are not good, use avatar's bone dir vector?
                        boneOrientVecs[skelIdx][bnIdx] = Eigen::Vector3d(avtrBoneOrientVecs[bnIdx]);
                        boneAdjOrientVecs[skelIdx][bnIdx] = Eigen::Vector3d(avtrBoneAdjOrientVecs[bnIdx]);				
                    }
                }//if isAvtr else
            }//for each bone
            if(isAvtr){
                if (!STRM_Flags[_SK_AVTR_INIT_SET]){																					//if not set already, set displacements of avatar skel from root, to set positions for untracked joints of tracked skeletons - only do once
                    for(int jntIdx = 0; jntIdx < NUI_NumJnts; ++jntIdx){	avtrOrigRelDispFromRoot[jntIdx] = Eigen::Vector3d(jntIdx == jIdx_ROOT ? vec3_zero : avtrOrigJntFrame[jntIdx] - avtrOrigJntFrame[jIdx_ROOT]);	}		//vector displacement for original joint locations
                }//if isavatar and haven't already set initial vals
                Eigen::Vector3d frwrd = avtrBoneOrientVecs[bIdx_HipC_HipR].cross(avtrBoneOrientVecs[bIdx_HipC_HipL]);				//frwrd vector is bone vec from root to right hip crossed with root to left hip of avatar bone vecs (modified from last IK of avatar to cleaned Kin Skel joints
                cSkelForward = (Eigen::Vector3d(frwrd(0),0,frwrd(2))).normalized();													//use normalized projection onto the x-z plane - use to build knee and foot dir vecs
                STRM_Flags[_SK_AVTR_INIT_SET] = true;																				//all original bone lengths and vectors will be set after first pass for avatar - don't reset every pass
            }//if isavtr							
        }//buildBoneVectorArray

        //build joint angles based on kinect skel - very limited usefulness - kinect joints often bad, even tracked
        void KinSkelHandler::buildKinSkelJointAngles(const vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& jointLocs){	
            for(int jntIdx = 0; jntIdx < NUI_NumJnts; ++jntIdx){
                Eigen::Vector3d joint(jointLocs[jntIdx]), jntA(jointLocs[jntPrev[jntIdx]]), jntB(jointLocs[jntNext[jntIdx]]);
                if((jntIdx == jntPrev[jntIdx]) || (jntPrev[jntIdx] == jntNext[jntIdx]) || (jntNext[jntIdx] == jntIdx)){				//if root or end joint, angle is 0 for now
                    skelJointAnglesGlobal[ctlSkel][jntIdx] = Eigen::Vector4d(0,0,0,1); 
                } else {
                    Eigen::Vector3d vA(jntA - joint), vB(jntB - joint);
                    vA.normalize();
                    vB.normalize();
                    Eigen::Vector3d vAxvB = vA.cross(vB);
                    skelJointAnglesGlobal[ctlSkel][jntIdx] = Eigen::Vector4d(vAxvB(0),vAxvB(1),vAxvB(2),acos(vA.dot(vB)));
                }
            }//for each joint 
        }//build vectors of local and global joint angles
	
        //calculate the y value on the floor for given x and z values - be sure to modify based on multipliers and offset
        double KinSkelHandler::calcFloorY(double xVal, double zVal){ 
            double yVal = -(((curFlrPlane[0] * xVal) + (curFlrPlane[2] * zVal) + curFlrPlane[3])/(0 == curFlrPlane[1] ? 1 :  curFlrPlane[1]));									//no div by 0 - would only happen if vertical "floor"
            yVal = (SK_JNT_MULT[1] * yVal) + SK_JNT_OFF[1];
            return yVal;
        }//calcFloorY

        //find min and max values of potential skeleton tracking by setting checking whether a joint is tracked or not
        void KinSkelHandler::setClipVals(int skelIdx, int jntIdx){
            Eigen::Vector3d jntLoc = Eigen::Vector3d( kinSkelJointLocs[skelIdx][jntIdx]);
            for(int idx = 0; idx <3; ++idx){			//for x, y, z
                if(jntLoc(idx) > MaxSkelClip[idx][idx]){
                    MaxSkelClip[idx][0] = jntLoc(0);
                    MaxSkelClip[idx][1] = jntLoc(1);
                    MaxSkelClip[idx][2] = jntLoc(2);
                } else if(jntLoc(idx)< MinSkelClip[idx][idx]){
                    MinSkelClip[idx][0] = jntLoc(0);
                    MinSkelClip[idx][1] = jntLoc(1);
                    MinSkelClip[idx][2] = jntLoc(2);
                }
            }
            //determine y min vals based on skel floor plane curFlrPlane - plug in min and max values for x and z to find floor y's for them
            FloorCrnrsY[0] = calcFloorY(MinSkelClip[0][0], MaxSkelClip[2][2]);					// FloorCrnrsY idx : 0 minXmaxZ, 1 minXminZ, 2 maxXminZ, 3 maxXmaxZ
            FloorCrnrsY[1] = calcFloorY(MinSkelClip[0][0], MinSkelClip[2][2]);
            FloorCrnrsY[2] = calcFloorY(MaxSkelClip[0][0], MinSkelClip[2][2]);
            FloorCrnrsY[3] = calcFloorY(MaxSkelClip[0][0], MaxSkelClip[2][2]);
        }//setClipVals

        //repairs erroneous leg orientations and to attempt to enable accurate, reliable squatting
        //sometimes kinect will return a knee joint that it says is accurately tracked that is around the skel's chest.  this can happen if the knee is occluded, or the user is sitting.
        //with a reasonably accurate set of hip joints, and an accurate representation of the ground, we can determine a decent kneejoint location.
        //both knees should bend the same amount
        void KinSkelHandler::cndLegJoints(int skelIdx, double hDiff){//avtrJntFrame
            double kneeSprdAngle = calcKneeSprdAngle(skelIdx);
            //kneel - use foot location and delta root from base root location to determine how low we go.
            //then find new knee location, rotating to the left and right based on how much further knees are apart than hips.
            double totLegLen = avtrOrigBoneSize[bIdx_HipL_KneeL] + avtrOrigBoneSize[bIdx_KneeL_AnkleL],
                bentLegLen = totLegLen - hDiff,
                thighHDiff = (avtrOrigBoneSize[bIdx_HipL_KneeL]/totLegLen) * (bentLegLen),							//thigh height after bend
                shinHDiff = (avtrOrigBoneSize[bIdx_KneeL_AnkleL]/totLegLen) * (bentLegLen),							//shin height after bend
                kneeDispVal = ((avtrOrigBoneSize[bIdx_HipL_KneeL] < thighHDiff) || (thighHDiff < 0) ?				//distance of knee from straight leg in x-z plane
                               0 : sqrt((avtrOrigBoneSize[bIdx_HipL_KneeL] * avtrOrigBoneSize[bIdx_HipL_KneeL]) - (thighHDiff * thighHDiff)));	

            //clog<<"KinSkelHandler : ----------- knee spread angle : "<<kneeSprdAngle<<" leg len "<<totLegLen<<" hDiff "<<hDiff<<std::endl;

            Eigen::Matrix3d kLRot = mat3_zero, kRRot = mat3_zero;															//this is just rotation around y axis - for non-vertical floor normals, need to get camera tilt angle and proj onto there
            //cos for all rots around y axis
            kLRot(0,0) = cos(kneeSprdAngle);														
            kLRot(2,2) = kLRot(0,0);
            kRRot(0,0) = kLRot(0,0);
            kRRot(2,2) = kLRot(0,0);

            kLRot(1,1) = 1;
            kRRot(1,1) = 1;
            //sin for all rots
            kLRot(0,2) = -sin(kneeSprdAngle);													
            kLRot(2,0) = -1*kLRot(0,2);																			//change which is negative if angles are b-wards
            kRRot(0,2) = -1*kLRot(0,2);
            kRRot(2,0) =  kLRot(0,2);

            //knee direction in x-z vectors - cSkelForward is 0 in y dir
            Eigen::Vector3d kneeRDir = (Eigen::Vector3d(kLRot * cSkelForward)).normalized(), 
                kneeLDir = (Eigen::Vector3d(kRRot * cSkelForward)).normalized();														//vectors from leg for knee locations and foot locations from ankle - reverse l and r due to kinect mirror orientation

            Eigen::Vector3d footRDir = (Eigen::Vector3d(kneeRDir[0], 0, kneeRDir[2])).normalized(),												//unit knee dir projected on x-z plane
                footLDir = (Eigen::Vector3d(kneeLDir[0], 0, kneeLDir[2])).normalized();

            double ankRY = calcFloorY(lastKnownSkelJointLocs[skelIdx][jIdx_HipR](0),lastKnownSkelJointLocs[skelIdx][jIdx_HipR](2)) + abs(avtrOrigBoneOrientVecs[bIdx_AnkleR_FootR](1)), 
                ankLY =    calcFloorY(lastKnownSkelJointLocs[skelIdx][jIdx_HipL](0),lastKnownSkelJointLocs[skelIdx][jIdx_HipL](2)) + abs(avtrOrigBoneOrientVecs[bIdx_AnkleL_FootL](1)),
                footRY =   calcFloorY(lastKnownSkelJointLocs[skelIdx][jIdx_HipR](0) + footRDir(0),lastKnownSkelJointLocs[skelIdx][jIdx_HipR](2) + footRDir(2)),
                footLY =   calcFloorY(lastKnownSkelJointLocs[skelIdx][jIdx_HipL](0) + footLDir(0),lastKnownSkelJointLocs[skelIdx][jIdx_HipL](2) + footLDir(2));

            //set ankles to be under hip joints and above ground by original displacement
            lastKnownSkelJointLocs[skelIdx][jIdx_AnkleR] = Eigen::Vector3d(lastKnownSkelJointLocs[skelIdx][jIdx_HipR](0), ankRY, lastKnownSkelJointLocs[skelIdx][jIdx_HipR](2));
            lastKnownSkelJointLocs[skelIdx][jIdx_AnkleL] = Eigen::Vector3d(lastKnownSkelJointLocs[skelIdx][jIdx_HipL](0), ankLY, lastKnownSkelJointLocs[skelIdx][jIdx_HipL](2));
            //set feet to be on ground, displaced from ankles by original avatar bone size, in direction of knee dir projected on ground
            lastKnownSkelJointLocs[skelIdx][jIdx_FootR] = lastKnownSkelJointLocs[skelIdx][jIdx_AnkleR] + (footRDir * avtrOrigBoneSize[bIdx_AnkleR_FootR]);
            lastKnownSkelJointLocs[skelIdx][jIdx_FootL] = lastKnownSkelJointLocs[skelIdx][jIdx_AnkleL] + (footLDir * avtrOrigBoneSize[bIdx_AnkleL_FootL]);
            //set knee location to be 
            lastKnownSkelJointLocs[skelIdx][jIdx_KneeR] = Eigen::Vector3d(kneeRDir(0) * kneeDispVal, shinHDiff + ankRY, kneeRDir(2) * kneeDispVal);
            lastKnownSkelJointLocs[skelIdx][jIdx_KneeL] = Eigen::Vector3d(kneeLDir(0) * kneeDispVal, shinHDiff + ankLY, kneeLDir(2) * kneeDispVal);
            //set tracking state to tracked
            kinSkelJointState[skelIdx][jIdx_FootR] = 2;
            kinSkelJointState[skelIdx][jIdx_FootL] = 2;
            kinSkelJointState[skelIdx][jIdx_KneeR] = 2;
            kinSkelJointState[skelIdx][jIdx_KneeL] = 2;
            //rebuild bone direction vectors based on new locations of foot, ankle and knee
            buildBoneVectorArray(skelIdx, false);																								//build arrays of kin skel bone vectors		

        }//cndLegJoints

        //calculates the angle between the legs in radians
        double KinSkelHandler::calcKneeSprdAngle(int skelIdx){
            double kneeSprdAngle, dotVal;
            if((1 <= boneTrackedState[skelIdx][bIdx_HipL_KneeL]) && (1 <= boneTrackedState[skelIdx][bIdx_HipR_KneeR])){						//if both tracked then set angle to be angle between these bones
                dotVal = boneOrientVecs[skelIdx][bIdx_HipL_KneeL].dot(boneOrientVecs[skelIdx][bIdx_HipR_KneeR]);
                //clog<<"KinSkelHandler : ----------- knee spread angle bone : "<<dotVal<<std::endl;
            } else {
                dotVal = avtrBoneOrientVecs[bIdx_HipL_KneeL].dot(avtrBoneOrientVecs[bIdx_HipR_KneeR]);
                //clog<<"KinSkelHandler : ----------- knee spread angle av bone : "<<dotVal<<std::endl;
            }
            kneeSprdAngle = acos(dotVal);
            return kneeSprdAngle;
        }//calcKneeAngle		

        //this method starts at the root and uses the bone vectors from the kinect skel to use as directions 
        //but restricts the length of each subsequent "bone" to be equal to the lengths of the 
        //corresponding bone in the avatar skel (so it scales the kinect skel to the same dimensions as the avatar skel)
        void KinSkelHandler::resizeKinSkel(int skelIdx){
            Eigen::Vector3d adjOrientVec;
            //need to put feet on "floor" after this is calculated
            for(int limb = 0; limb < NumLimbs; ++limb){
                int bStrtIDX = (((limb == 1) || (limb == 2)) ? 2 : 0 );															//for limbs 1 and 2, start at center shoulder to l/r shldr, not root (redundant limbs, root to spine, spine to shldr c)
                for (int bnIdx = bStrtIDX; bnIdx < NumBnsLmbPth; ++bnIdx){
                    int boneIDX = bnRtPthToTip[limb][bnIdx];																	//index of bone in bone vector structs
                    if (boneIDX == -1){	break;}																					//done with this limb
                    int jointModIDX = jntRtPthToTip[limb][bnIdx + 1];															//idx of joint we are modifying - never modify root, always 1 less bone on path than joint, if -1 then exit this iter of loop - done with bone
                    //want newJoint Position to be old prevjoint + (norm( bonevector[boneIDX]) * avtrOrigBoneSize[boneIDX]) in dir of new joint
                    adjOrientVec = Eigen::Vector3d (boneAdjOrientVecs[skelIdx][boneIDX]);															//size of bone in direction of tracked joints
                    lastKnownSkelJointLocs[skelIdx][jointModIDX] = lastKnownSkelJointLocs[skelIdx][jntPrev[jointModIDX]] + adjOrientVec;			//bone orientation vectors go from  tip toward root, so subtract
                }//for each bone in limb path
            }//for each limb
            //now need to lower every joint so that lowest foot is on floor if interpolated crouch enabled
            if(STRM_Flags[_SK_INT_CROUCH]){		
                clog<<"\tKinSkelHandler : Handling Crouch"<<std::endl;
                double lwrAmt = 0;
                if(lastKnownSkelJointLocs[skelIdx][jIdx_FootR][1] < lastKnownSkelJointLocs[skelIdx][jIdx_FootL][1]){						//find lowest foot, either left or right, and use dist from lowest to floor to determine skeleton lowering amount
                    lwrAmt = (lastKnownSkelJointLocs[skelIdx][jIdx_FootR](1) - calcFloorY(lastKnownSkelJointLocs[skelIdx][jIdx_FootR](0),lastKnownSkelJointLocs[skelIdx][jIdx_FootR](2)));
                } else {
                    lwrAmt = (lastKnownSkelJointLocs[skelIdx][jIdx_FootL](1) - calcFloorY(lastKnownSkelJointLocs[skelIdx][jIdx_FootL](0),lastKnownSkelJointLocs[skelIdx][jIdx_FootL](2)));
                }
                if(abs(lwrAmt) > kinHNDL_epsVal){ 
                    //lower every joint by lwrAmt
                    for(int jIdx = 0; jIdx < NUI_NumJnts; ++jIdx){	lastKnownSkelJointLocs[skelIdx][jIdx](1) -= lwrAmt;	}
                }
            }//if hdiff ! 0
        }//cndKinSkel
	
        //converts actual skel frame data to screen/world location - skeletonpoint contains x,y,z,1 where x,y,z are dist from sensor in meters
        Eigen::Vector3d KinSkelHandler::buildSkelJointLocation(Vector4 sP, int idx){//SK_JNT_MULT
            Eigen::Vector3d retVal((SK_JNT_MULT[0] * sP.x) + SK_JNT_OFF[0], 
                                   (SK_JNT_MULT[1] * sP.y) + SK_JNT_OFF[1], 
                                   (SK_JNT_MULT[2] * sP.z) + SK_JNT_OFF[2]);						//point in skel space -> display appropriately
            return retVal;
        }//buildSkelJointLocation

        //return appropriate kinect flag (or 0) for this stream corresponding to current setting of state machine flag 
        unsigned long KinSkelHandler::getKHSMFlagMode(int idx, bool trackOrFrame){//different flags for either tracking or frame - true->track, false->frame
            bool val = owningKH->getFlag(idx);
            switch (idx){
            case _KH_SEATED_IDX			: {
                return (val  ? (trackOrFrame ? NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT : NUI_SKELETON_FRAME_FLAG_SEATED_SUPPORT_ENABLED ): 0l);}
            case _KH_NEAR_IDX			: {
                return (val  ? NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE : 0l);}
            case _KH_SKEL_TRACKED_IDX		: {
                return (val  ? NUI_SKELETON_TRACKED : 0l);}
            case _KH_SKEL_ROOTONLY_IDX	: {
                return (val  ? NUI_SKELETON_POSITION_ONLY : 0l);}
            case _KH_SKEL_NONEFOUND_IDX		: {
                return NUI_SKELETON_NOT_TRACKED;}		//0
            }//switch
            return 0l;
        }//getKHSMFlagMode

        //handle this stream's state machine
        bool KinSkelHandler::setFlag(int idx, bool val){
            this->STRM_Flags[idx] = val;
            switch (idx){
            case _SK_SEATED_IDX			:{break;}
            case _SK_NEAR_IDX			:{break;}
            case _SK_AVTR_INIT_SET		:{break;}
            case _SK_DISP_AVTR_SKEL		:{break;}
            case _SK_DISP_FLTR_SKEL		:{break;}
            case _SK_DISP_NSY_SKEL		:{break;}
            case _SK_CAM_TILT_SET		:{break;}
            case _SK_INT_CROUCH			:{break;}
            case _SK_SAVE_JDATA			:{break;}
            default : {break;}
            }//switch	
            return true;
        }//SetFlags

        //returns array of strings holding joint angle values - NUI_NumJnts == number of joints in skeleton
        vector<string> KinSkelHandler::getJointAngleStrs(){
            vector<string> result(NUI_NumJnts);	
            if(validCtlSkel()){
                buildKinSkelJointAngles(lastKnownSkelJointLocs[ctlSkel]);		
                for(int i = 0; i < result.size(); ++i){
                    std::stringstream ss(stringstream::in | stringstream::out);
                    ss<<kinSkelJointNames[i];
                    ss<<" (";
                    ss.precision(2);
                    ss<<(skelJointAnglesGlobal[ctlSkel][i][0]);
                    ss<<",";
                    ss<<skelJointAnglesGlobal[ctlSkel][i][1];
                    ss<<",";
                    ss<<skelJointAnglesGlobal[ctlSkel][i][2];
                    ss<<"| ";
                    ss<<skelJointAnglesGlobal[ctlSkel][i][3];
                    ss<<")";			
                    std::string tmpVal = ss.str();
                    result[i] = tmpVal;
                }//for each joint angle
            } else {
                for(int i = 0; i < result.size(); ++i){
                    std::stringstream ss(stringstream::in | stringstream::out);
                    ss<<kinSkelJointNames[i];
                    ss<<" (-,-,-| -)";
                    std::string tmpVal = ss.str();
                    result[i] = tmpVal;
                }
            }
            return result;
        }//getJointAngleStrs

        vector<string> KinSkelHandler::getAvtrBoneLenVecStrs(){
            vector<string> result(NUI_NumBones);
            if(STRM_Flags[_SK_AVTR_INIT_SET]){					
                for(int i = 0; i < result.size(); ++i){
                    double aLen = avtrOrigBoneSize[i];
                    std::stringstream ss(stringstream::in | stringstream::out);
                    ss<<kinSkelBoneNames[i];
                    ss<<" = ";
                    ss.precision(2);
                    ss<<(aLen);
                    std::string tmpVal = ss.str();
                    result[i] = tmpVal;
                }//for each bone
            }//if STRM_Flags[_SK_AVTR_BN_SET]
            return result;
        }//getAvtrBoneLenVecStrs 

        vector<string> KinSkelHandler::getBestKinSkelJointLocsStrs(){
            vector<string> result(NUI_NumJnts, "0");
            if(validCtlSkel()){
                for(int i = 0; i < result.size(); ++i){//lastKnownSkelJointLocs
                    std::stringstream ss(stringstream::in | stringstream::out);
                    ss<<kinSkelJointNames[i];
                    ss<<" = (";
                    ss.precision(2);
                    ss<<(lastKnownSkelJointLocs[ctlSkel][i][0]);
                    ss<<", ";
                    ss.precision(2);
                    ss<<lastKnownSkelJointLocs[ctlSkel][i][1];
                    ss<<", ";
                    ss.precision(2);
                    ss<<lastKnownSkelJointLocs[ctlSkel][i][2];
                    ss<<")";			
                    std::string tmpVal = ss.str();
                    result[i] = tmpVal;
                }//for each joint
            } else {
                for(int i = 0; i < result.size(); ++i){
                    std::stringstream ss(stringstream::in | stringstream::out);
                    ss<<kinSkelJointNames[i];
                    ss<<" = (0, 0, 0)";
                    std::string tmpVal = ss.str();
                    result[i] = tmpVal;
                }//for each joint
            }
            return result;
        }//getBestKinSkelJointLocsStrs 

        void KinSkelHandler::dispClipVals(){
            //clog<<std::endl;
            for(int idx = 0; idx < 3; ++idx){
                stringstream ss;
                ss<<MaxSkelClip[idx][0];
                ss<<",";
                ss<<MaxSkelClip[idx][1];
                ss<<",";
                ss<<MaxSkelClip[idx][2];
                string tmp = ss.str();
                //clog<<"before clip : Max skel coords idx " << idx<<" : ("<<tmp<<")"<<std::endl;
                stringstream ss2;
                ss2<<MinSkelClip[idx][0];
                ss2<<",";
                ss2<<MinSkelClip[idx][1];
                ss2<<",";
                ss2<<MinSkelClip[idx][2];
                tmp = ss2.str();
                //clog<<"before clip : Min skel coords idx " << idx<<" : ("<<tmp<<")"<<std::endl;
            }
            //clog<<std::endl;
        }//dispClipVals

        //	//records all displacements, finds average and maximum over a set period of time
        //void KinSkelHandler::findJointDispVals(int jntIdx, double disp){
        //		//accumulate displacement
        //	jntDispValidSum[jntIdx] += disp;
        //	jntDispValidMax[jntIdx] = max(disp, jntDispValidMax[jntIdx]);
        //	++jntDispIter[jntIdx];										//per-joint count of valid joint displacements

        //	if (jntDispIter[jntIdx] > jntDispMaxIter){
        //		jntDispAvgAra[jntIdx].pop_back();
        //		jntDispMaxAra[jntIdx].pop_back();
        //		double dispAvg = jntDispValidSum[jntIdx]/(1.0 * jntDispMaxIter);
        //		jntDispAvgAra[jntIdx].push_front(dispAvg);
        //		jntDispMaxAra[jntIdx].push_front(jntDispValidMax[jntIdx]);
        //		//clear accumulators
        //		jntDispValidSum[jntIdx] = 0;
        //		jntDispValidMax[jntIdx] = 0;
        //		jntDispIter[jntIdx] = 0;
        //		jntDispAccumIter[jntIdx]++;
        //		clog<<std::endl;
        //		clog<<"KinSkelHandler Log : ----displacement tracking iter : "<<jntDispAccumIter[jntIdx]<<" for joint : "<<kinSkelJointNames[jntIdx]<<std::endl;
        //	}//if finished 1 sweep
        //	if(jntDispAccumIter[jntIdx] > jntPosAraDpth){//once we have more than jntPosAraDpth, print out results
        //		clog<<std::endl;
        //		for(int iter = 0; iter < jntDispAccumIter[jntIdx]; ++iter){
        //			clog<<"KinSkelHandler Log : ***---- displacement tracking results : avg disp : "<<jntDispAvgAra[jntIdx][iter]<<" max displacement : "<<jntDispMaxAra[jntIdx][iter]<<" for joint : "<<kinSkelJointNames[jntIdx]<<std::endl;
        //		}//for each result
        //		clog<<std::endl;

        //		for(int dIdx = 0; dIdx < jntPosAraDpth; ++dIdx){
        //			jntDispAvgAra[jntIdx].push_front(0);
        //			jntDispMaxAra[jntIdx].push_front(0);
        //		}
        //		jntDispAccumIter[jntIdx] = 0;
        //	}//if we've counted enough
        //}//findJointDispVals

        //returns quality of a bone - if both joints tracked, bone is tracked(2) if either joint inferred, bone is inferred(1) if neither tracked nor inferred then bone is not tracked (0)
        int KinSkelHandler::boneQuality(int bnIdx){
            if (!validCtlSkel()) return 0;
            if ((NUI_SKELETON_POSITION_TRACKED == kinSkelJointState[ctlSkel][bnJoints[bnIdx][0]]) && (NUI_SKELETON_POSITION_TRACKED == kinSkelJointState[ctlSkel][bnJoints[bnIdx][1]])) { return NUI_SKELETON_POSITION_TRACKED;}		//both joints tracked, bone is tracked
            if ((NUI_SKELETON_POSITION_INFERRED == kinSkelJointState[ctlSkel][bnJoints[bnIdx][0]]) || (NUI_SKELETON_POSITION_INFERRED == kinSkelJointState[ctlSkel][bnJoints[bnIdx][1]])) { return NUI_SKELETON_POSITION_INFERRED;}
            return NUI_SKELETON_POSITION_NOT_TRACKED;//neither joint tracked or inferred, 
        }//boneQuality

        vector<double> KinSkelHandler::getClippingBox(){
            //min/max x : idx 0,1
            //min/max y : idx 2,3			-get 4 values for min y, take avg
            //min/max z	: idx 4,5
            vector<double> box(6);
            box[0] = MinSkelClip[0][0];
            box[1] = MaxSkelClip[0][0];
            box[2] = (FloorCrnrsY[0] + FloorCrnrsY[1] + FloorCrnrsY[2] + FloorCrnrsY[3])/4.0;
            box[3] = MaxSkelClip[1][1];
            box[4] = MinSkelClip[2][2];
            box[5] = MaxSkelClip[2][2];
            return box;		
        }//getClippingBox


        //potential inlines - move to h?
        bool KinSkelHandler::jntTrk(int state){return (NUI_SKELETON_POSITION_TRACKED == state);}
        bool KinSkelHandler::jntInf(int state){return (NUI_SKELETON_POSITION_INFERRED == state);}
        bool KinSkelHandler::jntUnk(int state){return (NUI_SKELETON_POSITION_NOT_TRACKED == state);}
        bool KinSkelHandler::bothJointsUnknown(int joint0State, int joint1State){return ((NUI_SKELETON_POSITION_NOT_TRACKED == joint0State) && (NUI_SKELETON_POSITION_NOT_TRACKED == joint1State));}	
        bool KinSkelHandler::eitherJointNotTracked(int joint0State, int joint1State){return ((NUI_SKELETON_POSITION_TRACKED != joint0State) || (NUI_SKELETON_POSITION_TRACKED != joint1State));}//either joint is inferred or not tracked
        bool KinSkelHandler::bothJointsTracked(int joint0State, int joint1State){return ((NUI_SKELETON_POSITION_TRACKED == joint0State) && (NUI_SKELETON_POSITION_TRACKED == joint1State));}
        bool KinSkelHandler::eitherJointInferred(int joint0State, int joint1State){return ((NUI_SKELETON_POSITION_INFERRED == joint0State) || (NUI_SKELETON_POSITION_INFERRED == joint1State));}
        vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinSkelHandler::getBoneOrientVecs(){	return  (validCtlSkel() ?  boneOrientVecs[ctlSkel] : vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(NUI_NumJnts,vec3_zero));}
        vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinSkelHandler::getLastKnownSkelJointLocs(){	return (validCtlSkel() ? lastKnownSkelJointLocs[ctlSkel] :  vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(NUI_NumJnts,vec3_zero));	}		
        vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinSkelHandler::getKinSkelNoisyMarkers(){	return (validCtlSkel() ? currBaseSkelJointLocs[ctlSkel] :  vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(NUI_NumJnts,vec3_zero));	}		

        vector<Vector4> KinSkelHandler::getCurrRawSkelJointLocs(){	
            if(validCtlSkel()){		return currRawSkelJointLocs[ctlSkel];		}
            Vector4 tmpVec = Vector4();
            return vector<Vector4>(NUI_NumJnts,tmpVec);
        }

        vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> KinSkelHandler::getJointVels(){
            if(validCtlSkel()){		return jntVels; }
            Eigen::Vector3d tmpVec = Eigen::Vector3d(0,0,0);
            deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> tmpDeque = deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(jntPosAraDpth,tmpVec);
            return vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>>(NUI_NumJnts, tmpDeque);
        }

        vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>> KinSkelHandler::getJointAccels(){
            if(validCtlSkel()){		return jntAccels; }
            Eigen::Vector3d tmpVec = Eigen::Vector3d(0,0,0);
            deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> tmpDeque = deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(jntPosAraDpth,tmpVec);
            return vector<deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>>(NUI_NumJnts, tmpDeque);
        }

        deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinSkelHandler::getJointVelsForJoint(int jntIdx){
            if(validCtlSkel()){		return deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> (jntVels[jntIdx]); }
            Eigen::Vector3d tmpVec = Eigen::Vector3d(0,0,0);
            return deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(jntPosAraDpth,tmpVec);
        }

        deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> KinSkelHandler::getJointAccelsForJoint(int jntIdx){
            if(validCtlSkel()){		return deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> (jntAccels[jntIdx]); }
            Eigen::Vector3d tmpVec = Eigen::Vector3d(0,0,0);
            return deque<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>(jntPosAraDpth,tmpVec);
        }

        vector<int>	KinSkelHandler::getCtlSkelJointState(){			return (validCtlSkel() ? kinSkelJointState[ctlSkel] : vector<int>(NUI_NumJnts));}
		
    }//namespace kinect
}//namespace rtql8
