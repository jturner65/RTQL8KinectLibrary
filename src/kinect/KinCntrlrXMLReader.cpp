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
#include "KinCntrlrXMLReader.h"

namespace rtql8{
    namespace kinect{
        KinCntrlrXMLReader::KinCntrlrXMLReader(KinectController* _kc, bool _autoUpd)
            : kc(_kc),autoUpd(_autoUpd), hndlrFlageChg(G_NumHndlrs, false), tmpKH_flags(),tmpKC_flags(), 
            tmpKUI_flags(), tmpAH_flags(), tmpDH_flags(), tmpIMH_flags(), tmpINH_flags(), tmpSKH_flags()    
        {
            initVars();
        }//cnstrctr

        KinCntrlrXMLReader::~KinCntrlrXMLReader(){
        }//dstrctr

        //read in configuration file for KinectController gui instantiation
        bool KinCntrlrXMLReader::readFile(const char* const filename){
            // Load xml and create Document
            clog << "KinCntrlrXMLReader::readFile()" << endl;
            clog << "filename = " << filename << endl;
            try{
                ticpp::Document doc(filename);
                doc.LoadFile();

                ticpp::Iterator< ticpp::Element > child;
                for ( child = child.begin( &doc ); child != child.end(); child++ ) {
                    std::string name;
                    child->GetValue(&name);
                    if (name == "page") {                               readUIPage(*child);                 }
                    else if (name == "constants")  {                    readConstants(*child);              }
                    else if (name == "stateControl"){                   readStateControl(*child);           }
                    else if (name == "displayStyle"){                   readDisplayStyle(*child);           }
                    else if (name == "class") {                         classElements.push_back( &(*child) ); }        //must be read before any display elements in config xml
                }
            }
            catch(ticpp::Exception e){
                clog << "KinCntrlrXMLReader::readFile NG : " << e.what() << endl;
                return false;
            }
            clog << "KinCntrlrXMLReader::readFile() OK" << endl;
            return true;
        }

        //read in various global variables used in kinect library
        void KinCntrlrXMLReader::readConstants(ticpp::Element& node){
            try {
                //need to load default constants before kinectHandler is instantiated, to get grammar file name
                //must start with current values in global vars, overwrite values if present in xml, re-saves all vals

                int tmp_RCD                 = _RCD;                                                             //type:int        cmt:reconnect retry countdown timer
                long tmpkinMaxTilt          = kinMaxTilt;                                                       //type:long       cmt: max tilt angle
                long tmpkinMinTilt          = kinMinTilt;                                                       //type:long       cmt: min tilt angle
                float tmpgripH              = DP_GRIPH_THRESH;                                                  //type:float      cmt: threshold plane for grip detection - height
                float tmpgripW              = DP_GRIPW_THRESH;                                                  //type:float      cmt: threshold plane for grip detection - width
                float tmpgripD              = DP_GRIPD_THRESH;                                                  //type:float      cmt: threshold plane for grip detection - depth
                int tmpMX_CNT_FLR_Y         = MX_CNT_FLR_Y;                                                     //type:int        cmt:max number of consecutive frames of same floor depth counted (if kinects interpolated floor changes, decrement until 0 then change, to act as smoothing)
                int tmpjntPosAraDpth        = jntPosAraDpth;                                                    //type:int        cmt:how many previous values for jnt pos,vel,accel we will keep around
                int tmpjntNumMaxInterp      = jntNumMaxInterp;                                                  //type:int        cmt:maximum interpolations before we just use most recent avatar value for joint ara location
                int tmpTRY_KIN_RCN          = TRY_KIN_RCN;                                                      //type:int        cmt:kinect controller reconnect retry countdown
                int tmpKC_hndEps_2D         = KC_hndEps_2D;                                                     //type:int        cmt:threshold amount hands need to move in xy to be detected as moving
                float tmpKinPshDist         = KinPshDist;                                                       //type:float      cmt: arbitrary; defines delta z to be considered push
                float tmpKinIK_eps          = KinIK_eps;                                                        //type:float      cmt:IK eps amt
                float tmpKinIK_minPrtrb     = KinIK_minPrtrb;                                                   //type:float      cmt:minimum timestep amount we will allow for any single iteration
                int tmpKinIK_numIters       = KinIK_numIters;                                                   //type:int        cmt:number of IK iterations to run per timer cycle
                float tmpKinSldrBrdr        = KinSldrBrdr;                                                      //type:float      cmt:graphical border around slider object
                float tmpUIobj_DragSens     = UIobj_DragSens;                                                   //type:float      cmt:sensitivity of slider bar drag - 0.0-1.0
                float tmpUIobj_CrankSens    = UIobj_CrankSens;                                                  //type:float      cmt:sensitivity of crank bar drag - 0.0-1.0
                float tmpLHandDragSens      = LHandDragSens;                                                    //type:float      cmt:left hand sensitivity setting for when dragging
                float tmpRHandDragSens      = RHandDragSens;                                                    //type:float      cmt:right hand sensitivity setting for when dragging
                float tmpLHandPushSensIn    = LHandPushSensIn;                                                  //type:float      cmt:left hand push in sensitivity to detect click
                float tmpRHandPushSensIn    = RHandPushSensIn;                                                  //type:float      cmt:left hand push in sensitivity to detect click
                float tmpLHandPushSensOut   = LHandPushSensOut;                                                 //type:float      cmt:right hand pull back sensitivity to disengage click
                float tmpRHandPushSensOut   = RHandPushSensOut;                                                 //type:float      cmt:right hand pull back sensitivity to disengage click
                float tmpUIobj_MinFontScale = UIobj_MinFontScale;                                               //type:float      cmt:min value for font scaling for ui object labels
                float tmpUIobj_MaxFontScale = UIobj_MaxFontScale;                                               //type:float      cmt:max value for font scaling for ui object labels

                wstring tmpGrammarFileNameStr = wstring(GrammarFileName);                                       //type:wstring const cmt:file name for speech recognition grammar
                float tmpHndTrkBxMult[6] = {HndTrkBxMult[0],HndTrkBxMult[1],
                                            HndTrkBxMult[2],HndTrkBxMult[3],
                                            HndTrkBxMult[4],HndTrkBxMult[5]};                                   //type:float[6]    cmt:ara of multipliers for relative hand to root tracking box//{0,2,.5,1.5,0,2};

                float tmpkinSkelDisp[3][3]={{kinSkelDisp[0][0],kinSkelDisp[0][1],kinSkelDisp[0][2]},
                                            {kinSkelDisp[1][0],kinSkelDisp[1][1],kinSkelDisp[1][2]},
                                            {kinSkelDisp[2][0],kinSkelDisp[2][1],kinSkelDisp[2][2]}};           //type:float[3][3] cmt:translation on screen for drawing raw kinect joint ara data noisy kin, filtered kin, avatar(postIK)

                int tmpLHandBx[4]={LHandBx[0],LHandBx[1],LHandBx[2],LHandBx[3]};                                //type:int[4]      cmt:ara of vals for left hand detect box : {max x, min x-max x, max y, min y-max y}
                int tmpRHandBx[4]={RHandBx[0],RHandBx[1],RHandBx[2],RHandBx[3]};                                //type:int[4]      cmt:ara of vals for right hand detect box : {max x, min x-max x, max y, min y-max y}
                float tmpSK_JNT_MULT[3]={SK_JNT_MULT[0],SK_JNT_MULT[1],SK_JNT_MULT[2]};                         //type:float[3]    cmt:multiplier used when converting joint coords from skel space to display space
                float tmpSK_JNT_OFF[3]={SK_JNT_OFF[0],SK_JNT_OFF[1],SK_JNT_OFF[2]};                             //type:float[3]    cmt:offset used when converting joint coords from skel space to display space
                double tmpjntSz[3]={jntSz[0],jntSz[1],jntSz[2]};                                                //type:double[3]   cmt:size of skel joint ara for display
                NUI_TRANSFORM_SMOOTH_PARAMETERS tmpskelSmoothing = skelSmoothing;                               //type:struct NUI_TRANSFORM_SMOOTH_PARAMETERS     cmt:used for kinect sdks internal calc of skeleton motion smoothing - see sdk for params
                Eigen::VectorXd tmpKinIK_weights(KinIK_weights);                                                //KinIK_weights     type:Eigen::VectorXd    cmt:weights for joints in IK skel, idx'ed by kin joint ara idx - currently 20 joints in kin skel

                //read configuration for constants here                
                ticpp::Iterator< ticpp::Element > iter;
                for ( iter = iter.begin( &node ); iter != iter.end(); iter++ ) {
                    ticpp::Element& child = (*iter);
                    std::string name = child.GetAttribute("name");
                    std::string val = child.GetAttribute("val");
                    //clog<<" name: "<<name<<"\tval :"<<val<<"\ttype:"<<type<<"\tcmt:"<<cmt<<std::endl;

                    if ("_RCD" == name){                        child.GetAttribute("val", &tmp_RCD);  }           
                    else if ("kinMaxTilt" == name){             child.GetAttribute("val", &tmpkinMaxTilt);}        
                    else if ("kinMinTilt" == name){             child.GetAttribute("val", &tmpkinMinTilt);}       
                    else if ("gripH" == name){                  child.GetAttribute("val", &tmpgripH);}             
                    else if ("gripW" == name){                  child.GetAttribute("val", &tmpgripW);}             
                    else if ("gripD" == name){                  child.GetAttribute("val", &tmpgripD);}             
                    else if ("MX_CNT_FLR_Y" == name){           child.GetAttribute("val", &tmpMX_CNT_FLR_Y);}     
                    else if ("jntPosAraDpth" == name){          child.GetAttribute("val", &tmpjntPosAraDpth);}     
                    else if ("jntNumMaxInterp" == name){        child.GetAttribute("val", &tmpjntNumMaxInterp);}   
                    else if ("TRY_KIN_RCN" == name){            child.GetAttribute("val", &tmpTRY_KIN_RCN);}       
                    else if ("KC_hndEps_2D" == name){           child.GetAttribute("val", &tmpKC_hndEps_2D);}      
                    else if ("KinPshDist" == name){             child.GetAttribute("val", &tmpKinPshDist);}        
                    else if ("KinIK_eps" == name){              child.GetAttribute("val", &tmpKinIK_eps);}         
                    else if ("KinIK_minPrtrb" == name){         child.GetAttribute("val", &tmpKinIK_minPrtrb);}    
                    else if ("KinIK_numIters" == name){         child.GetAttribute("val", &tmpKinIK_numIters);}    
                    else if ("KinSldrBrdr" == name){            child.GetAttribute("val", &tmpKinSldrBrdr);}       
                    else if ("UIobj_DragSens" == name){         child.GetAttribute("val", &tmpUIobj_DragSens);} 
                    else if ("UIobj_CrankSens" == name){        child.GetAttribute("val", &tmpUIobj_CrankSens);} 
                    else if ("LHandDragSens" == name){          child.GetAttribute("val", &tmpLHandDragSens);} 
                    else if ("RHandDragSens" == name){          child.GetAttribute("val", &tmpRHandDragSens);} 
                    else if ("LHandPushSensIn" == name){        child.GetAttribute("val", &tmpLHandPushSensIn);}
                    else if ("RHandPushSensIn" == name){        child.GetAttribute("val", &tmpRHandPushSensIn);}
                    else if ("LHandPushSensOut" == name){       child.GetAttribute("val", &tmpLHandPushSensOut);}
                    else if ("RHandPushSensOut" == name){       child.GetAttribute("val", &tmpRHandPushSensOut);}
                    else if ("UIobj_MinFontScale" == name){     child.GetAttribute("val", &tmpUIobj_MinFontScale);}
                    else if ("UIobj_MaxFontScale" == name){     child.GetAttribute("val", &tmpUIobj_MaxFontScale);}

                    //need to populate these individually 
                    else if ("GramarFileNameStr" == name){ 
                        int len, slength = (int)val.length() + 1;
                        len = MultiByteToWideChar(CP_ACP, 0, val.c_str(), slength, 0, 0); 
                        wchar_t* buf = new wchar_t[len];
                        MultiByteToWideChar(CP_ACP, 0, val.c_str(), slength, buf, len);
                        std::wstring stemp(buf);
                        delete[] buf;
                        std::wstring stmpPath(WRTQL8_DATA_PATH);
                        stmpPath.append(stemp);
                        //tmpGrammarFileNameStr = stmpPath.c_str();
                        tmpGrammarFileNameStr = wstring(stmpPath);
                        wclog<<"KinCntrlrXMLReader : Grammar File name : "<<tmpGrammarFileNameStr<<endl;
                    } 
                    else if ("kinSkelDisp" == name){
                        child.GetAttribute("nx",&tmpkinSkelDisp[0][0]);
                        child.GetAttribute("ny",&tmpkinSkelDisp[0][1]);
                        child.GetAttribute("nz",&tmpkinSkelDisp[0][2]);
                        child.GetAttribute("fx",&tmpkinSkelDisp[1][0]);
                        child.GetAttribute("fy",&tmpkinSkelDisp[1][1]);
                        child.GetAttribute("fz",&tmpkinSkelDisp[1][2]);
                        child.GetAttribute("ax",&tmpkinSkelDisp[2][0]);
                        child.GetAttribute("ay",&tmpkinSkelDisp[2][1]);
                        child.GetAttribute("az",&tmpkinSkelDisp[2][2]);                    
                    }  
                    else if ("HndTrkBxMult" == name){
                        child.GetAttribute("lx",&tmpHndTrkBxMult[0]);
                        child.GetAttribute("ly",&tmpHndTrkBxMult[1]);
                        child.GetAttribute("lz",&tmpHndTrkBxMult[2]);
                        child.GetAttribute("hx",&tmpHndTrkBxMult[3]);
                        child.GetAttribute("hy",&tmpHndTrkBxMult[4]);
                        child.GetAttribute("hz",&tmpHndTrkBxMult[5]);
                    }  
                    else if ("LHandBx" == name){
                        child.GetAttribute("x",&tmpLHandBx[0]);                   
                        child.GetAttribute("y",&tmpLHandBx[1]);                   
                        child.GetAttribute("z",&tmpLHandBx[2]);                   
                        child.GetAttribute("w",&tmpLHandBx[3]);  
                    }           

                    else if ("RHandBx" == name){
                        child.GetAttribute("x",&tmpRHandBx[0]);                   
                        child.GetAttribute("y",&tmpRHandBx[1]);                   
                        child.GetAttribute("z",&tmpRHandBx[2]);                   
                        child.GetAttribute("w",&tmpRHandBx[3]);  
                    } 
                    else if ("skelSmoothing" == name){
                        child.GetAttribute("a",&tmpskelSmoothing.fSmoothing);
                        child.GetAttribute("b",&tmpskelSmoothing.fCorrection);
                        child.GetAttribute("c",&tmpskelSmoothing.fPrediction);
                        child.GetAttribute("d",&tmpskelSmoothing.fJitterRadius);
                        child.GetAttribute("e",&tmpskelSmoothing.fMaxDeviationRadius);
                        //clog<<"skel smoothing vals : "<<tmpskelSmoothing.fSmoothing<<"|"<<tmpskelSmoothing.fCorrection<<"|"<<tmpskelSmoothing.fPrediction<<"|"<<tmpskelSmoothing.fJitterRadius<<"|"<<tmpskelSmoothing.fMaxDeviationRadius<<std::endl;
                    }     
                    else if ("SK_JNT_MULT" == name){
                        child.GetAttribute("x",&tmpSK_JNT_MULT[0]);
                        child.GetAttribute("y",&tmpSK_JNT_MULT[1]);
                        child.GetAttribute("z",&tmpSK_JNT_MULT[2]);
                    }  
                    else if ("SK_JNT_OFF" == name){
                        child.GetAttribute("x",&tmpSK_JNT_OFF[0]);
                        child.GetAttribute("y",&tmpSK_JNT_OFF[1]);
                        child.GetAttribute("z",&tmpSK_JNT_OFF[2]);
                    }        
                    else if ("jntSz" == name){
                        child.GetAttribute("x",&tmpjntSz[0]);
                        child.GetAttribute("y",&tmpjntSz[1]);
                        child.GetAttribute("z",&tmpjntSz[2]);
                    }  
                    else if ("KinIK_weights" == name){  
                        tmpKinIK_weights = Eigen::VectorXd(NUI_NumJnts);
			            child.GetAttribute("w_jIdx_HipC",&tmpKinIK_weights(jIdx_HipC));		
			            child.GetAttribute("w_jIdx_Spine",&tmpKinIK_weights(jIdx_Spine));	
			            child.GetAttribute("w_jIdx_ShldrC",&tmpKinIK_weights(jIdx_ShldrC));	
			            child.GetAttribute("w_jIdx_Head",&tmpKinIK_weights(jIdx_Head));		
			            child.GetAttribute("w_jIdx_ShldrL",&tmpKinIK_weights(jIdx_ShldrL));	
			            child.GetAttribute("w_jIdx_ElbowL",&tmpKinIK_weights(jIdx_ElbowL));	
			            child.GetAttribute("w_jIdx_WristL",&tmpKinIK_weights(jIdx_WristL));	
			            child.GetAttribute("w_jIdx_HandL",&tmpKinIK_weights( jIdx_HandL));	
			            child.GetAttribute("w_jIdx_ShldrR",&tmpKinIK_weights(jIdx_ShldrR));	
			            child.GetAttribute("w_jIdx_ElbowR",&tmpKinIK_weights(jIdx_ElbowR));	
			            child.GetAttribute("w_jIdx_WristR",&tmpKinIK_weights(jIdx_WristR));	
			            child.GetAttribute("w_jIdx_HandR",&tmpKinIK_weights(jIdx_HandR));	
			            child.GetAttribute("w_jIdx_HipL",&tmpKinIK_weights(jIdx_HipL));		
			            child.GetAttribute("w_jIdx_KneeL",&tmpKinIK_weights(jIdx_KneeL));	
			            child.GetAttribute("w_jIdx_AnkleL",&tmpKinIK_weights(jIdx_AnkleL));	
			            child.GetAttribute("w_jIdx_FootL",&tmpKinIK_weights(jIdx_FootL));	
			            child.GetAttribute("w_jIdx_HipR",&tmpKinIK_weights(jIdx_HipR));		
			            child.GetAttribute("w_jIdx_KneeR",&tmpKinIK_weights(jIdx_KneeR));	
			            child.GetAttribute("w_jIdx_AnkleR",&tmpKinIK_weights(jIdx_AnkleR));	
			            child.GetAttribute("w_jIdx_FootR",&tmpKinIK_weights(jIdx_FootR));
                    } 
                }
                kc->setKinLibGlobalConstants(tmp_RCD, tmpkinMaxTilt, tmpkinMinTilt, tmpGrammarFileNameStr, tmpgripH, tmpgripW,  tmpgripD,  tmpLHandBx,  tmpRHandBx, tmpskelSmoothing,      
                        tmpHndTrkBxMult, tmpMX_CNT_FLR_Y, tmpSK_JNT_MULT, tmpSK_JNT_OFF, tmpjntSz, tmpjntPosAraDpth, tmpjntNumMaxInterp, tmpTRY_KIN_RCN, tmpKC_hndEps_2D, 
                        tmpKinPshDist, tmpkinSkelDisp, tmpKinIK_eps, tmpKinIK_minPrtrb, tmpKinIK_numIters, tmpKinIK_weights, tmpKinSldrBrdr, tmpUIobj_DragSens, 
                        tmpUIobj_CrankSens,tmpLHandDragSens,tmpRHandDragSens,tmpLHandPushSensIn, tmpRHandPushSensIn, tmpLHandPushSensOut, tmpRHandPushSensOut,tmpUIobj_MinFontScale,tmpUIobj_MaxFontScale);
                clog << "KinCntrlrXMLReader:: Default values mapped successfully"<<endl;                                                                      
            }
            catch(ticpp::Exception e){
                clog << "KinCntrlrXMLReader::readConstants failed : " << e.what() << endl;
            }
        }//readConstants

        //read in initial state machine flags for the various streams
        void KinCntrlrXMLReader::readStateControl(ticpp::Element& node){  
            try {
                    //flags from xml files, used if being read before handlers are instantiated
                map<std::string, int> tmpKCspeechVals;                         //map string phrases to int idx's representing flag indexes to be set

                ticpp::Iterator< ticpp::Element > iter;
                for ( iter = iter.begin( &node ); iter != iter.end(); iter++ ) {
                    ticpp::Element& handler = (*iter);
                    std::string value;
                    handler.GetValue(&value);

                    std::string name = handler.GetAttribute("name");

                    ticpp::Iterator< ticpp::Element > child;
                    for ( child = child.begin( &handler ); child != child.end(); child++ ) {
                        bool boolVal;
                        ticpp::Element& flag = (*child);
                        std::string value;
                        handler.GetValue(&value);
                        std::string type = flag.GetAttribute("type");
                        std::string idx = flag.GetAttribute("idx");
                        flag.GetAttribute("val",&boolVal);
                        std::string tag = flag.GetAttribute("tag");

                        if (type == "flag"){                       
                            if("KinectHandler"==name){             tmpKH_flags[KCidxVals[idx]] = boolVal;  hndlrFlageChg[_G_KinHndlr] = true; }   
                            else if ("KinAudioHandler"==name){     tmpAH_flags[KCidxVals[idx]] = boolVal;  hndlrFlageChg[_G_KinAUHndlr] = true; }  
                            else if ("KinDepthHandler"==name){     tmpDH_flags[KCidxVals[idx]] = boolVal;  hndlrFlageChg[_G_KinDPHndlr] = true; }
                            else if ("KinImageHandler"==name){     tmpIMH_flags[KCidxVals[idx]] = boolVal; hndlrFlageChg[_G_KinIMHndlr] = true; }
                            else if ("KinInteractHandler"==name){  tmpINH_flags[KCidxVals[idx]] = boolVal; hndlrFlageChg[_G_KinINHndlr] = true; }   
                            else if ("KinSkelHandler"==name){      tmpSKH_flags[KCidxVals[idx]] = boolVal; hndlrFlageChg[_G_KinSKHndlr] = true; }
                            else if ("KinectController"==name){    tmpKC_flags[KCidxVals[idx]] = boolVal;  hndlrFlageChg[_G_KinCntrlr] = true; }   
                            else if ("KinectUIComponent"==name){   tmpKUI_flags[KCidxVals[idx]] = boolVal; hndlrFlageChg[_G_KinKUIHndlr] = true; }    
                       } else if(type == "speech"){                
                            kc->setKCspeechVals(tag, KCidxVals[idx]);               //only use kinect controller for this
                        }    
                    }
                    //set flag values if handler is instantiated
                    if(autoUpd){updateHandlerStateFlags();}
                }//for each handler
            }
            catch(ticpp::Exception e){
                clog << "KinCntrlrXMLReader::readStateControl failed : " << e.what() << endl;
            }
        }//readStateControl

        //this will update all the state flags for all the handlers once they've all be instantiated - needs to be called after any flags are loaded, so that they are changed.  
        //this is required because the xml initialization sets variables required by kinecthandler when it starts, and these need to be set after it has aleady started
        void KinCntrlrXMLReader::updateHandlerStateFlags(){
            clog<<"KinCntrlrXMLReader : Update system state flags"<<std::endl;
            if(hndlrFlageChg[_G_KinHndlr]){   for(map<int, bool>::iterator iter = tmpKH_flags.begin(); iter != tmpKH_flags.end(); ++iter){ kc->setKinectHandlerFlag(iter->first, iter->second);}}   
            if(hndlrFlageChg[_G_KinAUHndlr]){ for(map<int, bool>::iterator iter = tmpAH_flags.begin(); iter != tmpAH_flags.end(); ++iter){ kc->setKinAudioHandlerFlag(iter->first, iter->second);}}
            if(hndlrFlageChg[_G_KinDPHndlr]){ for(map<int, bool>::iterator iter = tmpDH_flags.begin(); iter != tmpDH_flags.end(); ++iter){ kc->setKinDepthHandlerFlag(iter->first, iter->second);}}
            if(hndlrFlageChg[_G_KinIMHndlr]){ for(map<int, bool>::iterator iter = tmpIMH_flags.begin(); iter != tmpIMH_flags.end(); ++iter){kc->setKinImageHandlerFlag(iter->first, iter->second);}}
            if(hndlrFlageChg[_G_KinINHndlr]){ for(map<int, bool>::iterator iter = tmpINH_flags.begin(); iter != tmpINH_flags.end(); ++iter){kc->setKinInteractHandlerFlag(iter->first, iter->second);}} 
            if(hndlrFlageChg[_G_KinSKHndlr]){ for(map<int, bool>::iterator iter = tmpSKH_flags.begin(); iter != tmpSKH_flags.end(); ++iter){kc->setKinSkelHandlerFlag(iter->first, iter->second);}}  
            if(hndlrFlageChg[_G_KinCntrlr]){  for(map<int, bool>::iterator iter = tmpSKH_flags.begin(); iter != tmpSKH_flags.end(); ++iter){kc->setFlag(iter->first, iter->second);}}
            if(hndlrFlageChg[_G_KinKUIHndlr]){for(map<int, bool>::iterator iter = tmpKUI_flags.begin(); iter != tmpKUI_flags.end(); ++iter){kc->setKinectUIComponentFlag(iter->first, iter->second);    }} 
            clog<<"KinCntrlrXMLReader : Finished setting system state flags from xml"<<std::endl;
        }//updateHandlerStateFlags

        //TODO : read style values - allow 1 style per object type (eventually set on page level?)
        //style describes color, potentially other UI component variables
        void KinCntrlrXMLReader::readDisplayStyle(ticpp::Element& node){
            //for each object type listed
            //read in color element values
            //set all objects of this type to be specified color
            
        }//style sheet values for all UI objects

        //read page attribute
        KinUIPage* KinCntrlrXMLReader::readUIPage(ticpp::Element& node) {
            try{
                KinUIPage* page = new KinUIPage();
                node.GetAttribute("name", &(page->name));
                // clog << "readUIPage: name = " << page->name; 
                if (node.HasAttribute("onload")) {
                    node.GetAttribute("onload", &(page->onload));
                    // clog << " onload = " << page->onload;
                }
                // clog << endl;
                
                ticpp::Iterator< ticpp::Element > iter;
                for ( iter = iter.begin( &node ); iter != iter.end(); iter++ ) {
                    ticpp::Element& child = (*iter);
                    std::string value;
                    child.GetValue(&value);
                    if (value == "button") {
                        KinUIButton* btn = readUIButton(child);
                        page->addUIButton(btn);
                    }
                    if (value == "slider") {
                        KinUISlider* sld = readUISlider(child);
                        page->addUISlider(sld);
                    }
                    if (value == "crank") {
                        KinUICrank* crnk = readUICrank(child);
                        page->addUICrank(crnk);
                    }
                    if (value == "imgbutton") {
                        KinUIImageButton* imgb = readUIImgButton(child);
                        page->addUIImgBtn(imgb);
                    }
                    if (value == "prgbar"){
                        KinUIPrgBar* pbr = readUIPrgBar(child);
                        page->addUIPrgBar(pbr);
                    }
                    if (value == "timer"){
                        KinUITimer* tmr = readUITimer(child);
                        page->addUITimer(tmr);
                    }
                    if (value == "textbox"){
                        KinUITextBox* tb = readUITextBox(child);
                        page->addUITextBox(tb);
                    }
                    if (value == "caption"){
                        KinUICaption* cap = readUICaption(child);
                        page->addUICaption(cap);
                    }
                }
                kc->addPage(page);
                if (page->name == "start") {
                    clog << "set as start page" << endl;
                    clog << "# components = " << page->numComponents() << endl;
                    kc->updateAsPage(page);
                }
                
                return page;
            }
            catch(ticpp::Exception e){
                clog << "KinCntrlrXMLReader::readUIFile NG : " << e.what() << endl;
            }
            return NULL;
        }

        //base component values :  x,y, width/height, hotspot boundary in x and y, on_click value, label of obj
        void KinCntrlrXMLReader::readBaseCmpAttr(ticpp::Element& node, KinUIComponent* cmp){
            try{
                int x = 50, y = 50, w = 50, h = 50;
                float hsBrdrX=50, hsBrdrY=50;
                std::string onclick = "", label="", moLabel="";
                if(node.HasAttribute("x")){node.GetAttribute("x", &x);}
                if(node.HasAttribute("y")){node.GetAttribute("y", &y);}
                if(node.HasAttribute("w")){node.GetAttribute("w", &w);}
                if(node.HasAttribute("h")){node.GetAttribute("h", &h);}
                if(node.HasAttribute("hsBrdrX")){node.GetAttribute("hsBrdrX",&hsBrdrX);}
                if(node.HasAttribute("hsBrdrY")){node.GetAttribute("hsBrdrY",&hsBrdrY);}
                if (node.HasAttribute("onclick")) {  node.GetAttribute("onclick", &onclick); }
                if (node.HasAttribute("moLabel")) {node.GetAttribute("moLabel", &moLabel);}
                node.GetTextOrDefault(&label,"def lbl");

                if (node.HasAttribute("x") && node.HasAttribute("y")) {
                    cmp->setLoc(x, y);
                }
                if (node.HasAttribute("w") && node.HasAttribute("h")) {
                    cmp->setDim(w, h);
                }
                if (node.HasAttribute("hsBrdrX") && node.HasAttribute("hsBrdrY")) {
                    cmp->setHotSpot(hsBrdrX,hsBrdrY);
                }
                if (label.length() > 0) {
                    cmp->setLabel(label);
                    cmp->setMSOverTxt(label);
                }
                if (node.HasAttribute("moLabel")) {cmp->setMSOverTxt(moLabel);}
                
                if (node.HasAttribute("onclick")) {
                    cmp->setOnClick(onclick);
                }

                
                KinUIImageButton* imgbtn = dynamic_cast<KinUIImageButton*>(cmp);
                if (imgbtn != NULL && node.HasAttribute("src")) {
                    std::string src = "";
                    node.GetAttribute("src", &src);
                    imgbtn->setImgSrcFileName(RTQL8_DATA_PATH + src);          
                }

            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readBaseCmpAttr NG : " << e.what() << endl;
                exit(0);
            }        
        }//readBaseCmpAttr

        //set color of object - separate for style sheet support
        void KinCntrlrXMLReader::readBaseCmpColor(ticpp::Element& node, KinUIComponent* cmp){
            try{
                double r = 1.0, g = 1.0, b = 1.0, a = 1.0;

                if(node.HasAttribute("r")){node.GetAttribute("r", &r);}
                else return;
                if(node.HasAttribute("g")){node.GetAttribute("g", &g);}
                else return;
                if(node.HasAttribute("b")){node.GetAttribute("b", &b);}
                else return;
                if(node.HasAttribute("a")){node.GetAttribute("a", &a);}
                else return;
                
                cmp->setColor(r, g, b, a);
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readBaseCmpColor NG : " << e.what() << endl;
                exit(0);
            }              
        }//readBaseCmpColor

        void KinCntrlrXMLReader::readClassAttrAndColor(ticpp::Element& node,
                                                       KinUIComponent* cmp) {
            if (node.HasAttribute("class") == false) {
                return;
            }
            std::string classnames;
            node.GetAttribute("class", &classnames);
            classnames += ";";

            std::string name;
            for (int i = 0; i < classnames.length(); i++) {
                char c = classnames[i];
                if (c == ';') {
                    if (name.length() > 0) {
                        readClassAttrAndColor(name.c_str(), cmp);
                    }
                    name = "";
                } else {
                    name += c;
                }
            }
        }

        void KinCntrlrXMLReader::readClassAttrAndColor(const char* const name,
                                                       KinUIComponent* cmp) {
            std::string lhs(name);

            for (int i = 0; i < classElements.size(); i++) {
                ticpp::Element& node =*(classElements[i]);
                std::string rhs("");
                node.GetAttribute("name", &rhs);
                if (lhs == rhs) {
                    readClassAttrAndColor(node, cmp);
                    readBaseCmpAttr (node, cmp);
                    readBaseCmpColor(node, cmp);
                }
            }
        }

        //configure read-only caption - similar to button but only displays text
        KinUICaption* KinCntrlrXMLReader::readUICaption(ticpp::Element& node){
            try{
                KinUICaption* cap = new KinUICaption(0, 0);
                readClassAttrAndColor(node, cap);
                readBaseCmpAttr(node, cap);
                readBaseCmpColor(node, cap);
                // clog << "readUICaption : " << *cap << endl;
                return cap;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUICaption NG : " << e.what() << endl;
                exit(0);
            }
            return NULL;
        }//readUICaption

        KinUITextBox* KinCntrlrXMLReader::readUITextBox(ticpp::Element& node){
            try{
                KinUITextBox* tb = new KinUITextBox(0, 0);
                readClassAttrAndColor(node, tb);
                readBaseCmpAttr(node, tb);
                readBaseCmpColor(node, tb);
                int maxChars = 10;                                                                       //max number of input characters == max length of txtVal
                if(node.HasAttribute("maxChars")){node.GetAttribute("maxChars", &maxChars);}
                tb->setMaxChars(maxChars);
                // clog << "readUITextBox : " << *tb << endl;
                return tb;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUITextBox NG : " << e.what() << endl;
                exit(0);
            }
            return NULL;
        }//readUITextBox

        //read in UI button object
        KinUIButton* KinCntrlrXMLReader::readUIButton(ticpp::Element& node) {
            try{
                KinUIButton* btn = new KinUIButton(0, 0);
                readClassAttrAndColor(node, btn);
                readBaseCmpAttr(node, btn);
                readBaseCmpColor(node, btn);
                // clog << "readUIButton : " << *btn << endl;
                return btn;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUIButton NG : " << e.what() << endl;
                exit(0);
            }
            return NULL;
        }//readUIButton

        //read in UI crank object
        KinUICrank* KinCntrlrXMLReader::readUICrank(ticpp::Element& node){
            try{
                KinUICrank* crnk = new KinUICrank(0, 0);

                int cdiv=0, cbarlen=0, cbarthick=0;
                float crange=0, cpos=0, cinit=0;
                std::string dir;

                readClassAttrAndColor(node, crnk);
                readBaseCmpAttr(node, crnk);
                readBaseCmpColor(node,crnk);

                if(node.HasAttribute("cinit")){node.GetAttribute("cinit",&cinit);}
                if(node.HasAttribute("crange")){node.GetAttribute("crange",&crange);}
                if(node.HasAttribute("cpos")){node.GetAttribute("cpos",&cpos);}
                if(node.HasAttribute("cdiv")){node.GetAttribute("cdiv",&cdiv);}
                if(node.HasAttribute("cbarlen")){node.GetAttribute("cbarlen",&cbarlen);}
                if(node.HasAttribute("cbarthick")){node.GetAttribute("cbarthick",&cbarthick);}
                if(node.HasAttribute("dir")){node.GetAttribute("dir",&dir);}

                if (dir == "1" || dir == "cw" || dir == "clockwise") {crnk->setIsClockWise(true);}    
                else {                                                crnk->setIsClockWise(false);}   

                crnk->setCrankVals(cinit,crange,cpos,cdiv,cbarlen,cbarthick);

                double min = 0.0, max = 1.0;
                if (node.HasAttribute("min")) {node.GetAttribute("min", &min);}
                if (node.HasAttribute("max")) {node.GetAttribute("max", &max);}
                crnk->setCrankMin(min);
                crnk->setCrankRng(max - min);

                // clog<<"readUIcrank : "<<*crnk<<endl;
                return crnk;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUICrank NG : " << e.what() << endl;
                exit(0);
            }
            return NULL;

        }//readUICrank

        //xml should be identical with button, except use imgbutton tag and src="<file name>" tag
        KinUIImageButton* KinCntrlrXMLReader::readUIImgButton(ticpp::Element& node){
            try{
                KinUIImageButton* ibtn = new KinUIImageButton(0, 0);
                std::string src = "";

                readClassAttrAndColor(node, ibtn);
                readBaseCmpAttr(node, ibtn);
                readBaseCmpColor(node, ibtn);

                if (node.HasAttribute("src")) {
                    node.GetAttribute("src", &src);                                                 //used to pick image to display.  for fixed (not animated) images only
                    // src = "smile.jpg";
                    //src =  src;
                    // cout << "src = " << src << endl;
                    ibtn->setImgSrcFileName(RTQL8_DATA_PATH +src);               
                }
                // clog << "readUIImageButton : " << *ibtn << endl;
                return ibtn;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUIImgButton NG : " << e.what() << endl;
                exit(0);
            }
            return NULL;
        
        }//readUIImgButton


        //process common elements of slider-based components - slider, prgbar, timer
        void KinCntrlrXMLReader::readBaseSliderAttr(ticpp::Element& node, KinUISlider* sld){
            try{

                int sdiv=0,
                    sbarlen=200,
                    sbarthick=20;
                float srange=100,spos=0;
                std::string dir="v", ltor="t";

                readClassAttrAndColor(node, sld);
                readBaseCmpAttr(node, sld);                                     //default values and base values for components - (x,y) loc, width/height, hotspot border in x and y, onclick, label
                readBaseCmpColor(node, sld);                                    //color for object - separate so that style sheet can be made and used to set up style/color for all objects

                if(node.HasAttribute("srange")){node.GetAttribute("srange",&srange);}
                if(node.HasAttribute("spos")){node.GetAttribute("spos",&spos);}
                if(node.HasAttribute("sdiv")){node.GetAttribute("sdiv",&sdiv);}
                if(node.HasAttribute("sbarlen")){node.GetAttribute("sbarlen",&sbarlen);}
                if(node.HasAttribute("sbarthick")){node.GetAttribute("sbarthick",&sbarthick);}
                if(node.HasAttribute("dir")){node.GetAttribute("dir",&dir);}
                if(node.HasAttribute("ltor")){node.GetAttribute("ltor",&ltor);}

                sld->setIsHoriz((dir == "1"     || dir == "h"   || dir == "horizontal")); 
                sld->setIsLToR((ltor == "1"     || ltor == "t"  || ltor == "true") );                                //slider increases left to right(horiz), or top to bottom(vert), or slider increases right to left(horiz), or bottom to top(vert),

                sld->setSlideVals(srange,spos,sdiv,sbarlen,sbarthick);

                double min = 0.0, max = 1.0;
                if (node.HasAttribute("min")) { node.GetAttribute("min", &min);}                            //min should always be below max - to change direction, use LToR
                if (node.HasAttribute("max")) { node.GetAttribute("max", &max);}

                sld->setSlideMin(min);
                sld->setSlideRng(max - min);
            }
            catch(ticpp::Exception e) {


            }
        }//readBaseSliderAttr

        KinUIPrgBar* KinCntrlrXMLReader::readUIPrgBar(ticpp::Element& node){
            try{
                KinUIPrgBar* prg = new KinUIPrgBar(0, 0);
                readBaseSliderAttr(node, prg);

                //int sdiv=0,
                //    sbarlen=200,
                //    sbarthick=20;
                //float srange=100,spos=0;                                                     
                //std::string dir="v",                                                                    //vertical or horizontal 
                //    ltor="t";                                                                           //increases left to right or right to left/top to bottom or bottom to top 

                //readClassAttrAndColor(node, prg);
                //readBaseCmpAttr(node, prg);                                                             //default values and base values for components - (x,y) loc, width/height, hotspot border in x and y, onclick, label
                //readBaseCmpColor(node, prg);                                                            //color for object - separate so that style sheet can be made and used to set up style/color for all objects

                //if(node.HasAttribute("srange")){node.GetAttribute("srange",&srange);}
                //if(node.HasAttribute("spos")){node.GetAttribute("spos",&spos);}
                //if(node.HasAttribute("sdiv")){node.GetAttribute("sdiv",&sdiv);}
                //if(node.HasAttribute("sbarlen")){node.GetAttribute("sbarlen",&sbarlen);}
                //if(node.HasAttribute("sbarthick")){node.GetAttribute("sbarthick",&sbarthick);}
                //if(node.HasAttribute("dir")){node.GetAttribute("dir",&dir);}
                //if(node.HasAttribute("ltor")){node.GetAttribute("ltor",&ltor);}

                //prg->setIsHoriz((dir == "1"     || dir == "h"   || dir == "horizontal")); 
                //prg->setIsLToR((ltor == "1"     || ltor == "t"  || ltor == "true") );                           //indicator increases left to right(horiz), or top to bottom(vert), or slider increases right to left(horiz), or bottom to top(vert),

                //prg->setSlideVals(srange,spos,sdiv,sbarlen,sbarthick);

                //double min = 0.0, max = 1.0;
                //if (node.HasAttribute("min")) { node.GetAttribute("min", &min);}                            //min should always be below max - to change direction, use LToR
                //if (node.HasAttribute("max")) { node.GetAttribute("max", &max);}

                //prg->setSlideMin(min);
                //prg->setSlideRng(max - min);

                std::string incr="0";                                                                           //whether or not the display for this prg bar should increase or decrease  

                if(node.HasAttribute("incr")){node.GetAttribute("incr",&incr);}
                prg->setIsIncr((incr == "1"     || incr == "t"  || incr == "true"));                            //dispalying increasing values or decreasing values (amt of progress, or remaining to do)

                // clog<<"readUIPrgBar : "<<*prg<<endl;
                return prg;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUIPrgBar NG : " << e.what() << endl;
                exit(0);
            }
            return NULL; 
        }//readUIPrgBar

        KinUITimer* KinCntrlrXMLReader::readUITimer(ticpp::Element& node){
            try{
                KinUITimer* tmr = new KinUITimer(0, 0);
                readBaseSliderAttr(node, tmr);

                //int sdiv=0,
                //    sbarlen=200,
                //    sbarthick=20;
                //float srange=100,spos=0;                                                     
                //std::string dir="v",                                                                    //vertical or horizontal 
                //    ltor="t";                                                                           //increases left to right or right to left/top to bottom or bottom to top 

                //readClassAttrAndColor(node, tmr);
                //readBaseCmpAttr(node, tmr);                                                             //default values and base values for components - (x,y) loc, width/height, hotspot border in x and y, onclick, label
                //readBaseCmpColor(node, tmr);                                                            //color for object - separate so that style sheet can be made and used to set up style/color for all objects

                //if(node.HasAttribute("srange")){node.GetAttribute("srange",&srange);}
                //if(node.HasAttribute("spos")){node.GetAttribute("spos",&spos);}
                //if(node.HasAttribute("sdiv")){node.GetAttribute("sdiv",&sdiv);}
                //if(node.HasAttribute("sbarlen")){node.GetAttribute("sbarlen",&sbarlen);}
                //if(node.HasAttribute("sbarthick")){node.GetAttribute("sbarthick",&sbarthick);}
                //if(node.HasAttribute("dir")){node.GetAttribute("dir",&dir);}
                //if(node.HasAttribute("ltor")){node.GetAttribute("ltor",&ltor);}

                //tmr->setIsHoriz((dir == "1"     || dir == "h"   || dir == "horizontal")); 
                //tmr->setIsLToR((ltor == "1"     || ltor == "t"  || ltor == "true") );                       //indicator increases left to right(horiz), or top to bottom(vert), or slider increases right to left(horiz), or bottom to top(vert),
 
                //tmr->setSlideVals(srange,spos,sdiv,sbarlen,sbarthick);

                //double min = 0.0, max = 1.0;
                //if (node.HasAttribute("min")) { node.GetAttribute("min", &min);}                            //min should always be below max - to change direction, use LToR
                //if (node.HasAttribute("max")) { node.GetAttribute("max", &max);}

                //tmr->setSlideMin(min);
                //tmr->setSlideRng(max - min);

                std::string incr="0";                                                                       //whether or not the display for this timer should increase or decrease  - elapsed time or time remaining (if countdown) 

                if(node.HasAttribute("incr")){node.GetAttribute("incr",&incr);}
                tmr->setIsIncr((incr == "1"     || incr == "t"  || incr == "true"));                        //displaying increasing values or decreasing values (elapsed time or time remaining (if countdown))

                // clog<<"readUITimer : "<<*tmr<<endl;
                return tmr;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUIPrgBar NG : " << e.what() << endl;
                exit(0);
            }
            return NULL; 
        }//readUIPrgBar

        KinUISlider* KinCntrlrXMLReader::readUISlider(ticpp::Element& node){
            try{
                KinUISlider* sld = new KinUISlider(0, 0);
                readBaseSliderAttr(node, sld);
                // clog<<"readUISlider : "<<*sld<<endl;
                return sld;
            }
            catch(ticpp::Exception e) {
                clog << "KinCntrlrXMLReader::readUISlider NG : " << e.what() << endl;
                exit(0);
            }
            return NULL;
        }//readUISlider

        //initialize variables
        void KinCntrlrXMLReader::initVars(){
            KCidxVals.insert(std::pair<std::string, const int>("_KH_SEATED_IDX", _KH_SEATED_IDX));
            KCidxVals.insert(std::pair<std::string, const int>("_KH_NEAR_IDX", _KH_NEAR_IDX));
            KCidxVals.insert(std::pair<std::string, const int>("_KH_DPTH_DISP_HND", _KH_DPTH_DISP_HND));
            KCidxVals.insert(std::pair<std::string, const int>("_KH_SKEL_CALC_HND", _KH_SKEL_CALC_HND));
            KCidxVals.insert(std::pair<std::string, const int>("_KH_DISP_DEBUG", _KH_DISP_DEBUG));

            KCidxVals.insert(std::pair<std::string, const int>("_DP_SEATED_IDX",_DP_SEATED_IDX));
            KCidxVals.insert(std::pair<std::string, const int>("_DP_NEAR_IDX",_DP_NEAR_IDX));	
            KCidxVals.insert(std::pair<std::string, const int>("_DP_PROC_HNDS",_DP_PROC_HNDS));	
            KCidxVals.insert(std::pair<std::string, const int>("_DP_DISP_HNDS",_DP_DISP_HNDS));	
            KCidxVals.insert(std::pair<std::string, const int>("_DP_BUILD_HNDIMG",_DP_BUILD_HNDIMG));

            KCidxVals.insert(std::pair<std::string, const int>("_SK_SEATED_IDX", _SK_SEATED_IDX));
            KCidxVals.insert(std::pair<std::string, const int>("_SK_NEAR_IDX", _SK_NEAR_IDX));
            KCidxVals.insert(std::pair<std::string, const int>("_SK_DISP_AVTR_SKEL", _SK_DISP_AVTR_SKEL));
            KCidxVals.insert(std::pair<std::string, const int>("_SK_DISP_FLTR_SKEL", _SK_DISP_FLTR_SKEL));
            KCidxVals.insert(std::pair<std::string, const int>("_SK_DISP_NSY_SKEL", _SK_DISP_NSY_SKEL));
            KCidxVals.insert(std::pair<std::string, const int>("_SK_CAM_TILT_SET", _SK_CAM_TILT_SET));
            KCidxVals.insert(std::pair<std::string, const int>("_SK_INT_CROUCH", _SK_INT_CROUCH));
            KCidxVals.insert(std::pair<std::string, const int>("_SK_SAVE_JDATA", _SK_SAVE_JDATA));

            KCidxVals.insert(std::pair<std::string, const int>("UIobjIDX_Display",UIobjIDX_Display));
            KCidxVals.insert(std::pair<std::string, const int>("UIobjIDX_DispHS",UIobjIDX_DispHS));
            KCidxVals.insert(std::pair<std::string, const int>("UIobjIDX_CanClick",UIobjIDX_CanClick));	
            KCidxVals.insert(std::pair<std::string, const int>("UIobjIDX_CanDrag",UIobjIDX_CanDrag));

            KCidxVals.insert(std::pair<std::string, const int>("_KC_USE_STRM_DPTH", _KC_USE_STRM_DPTH));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_USE_STRM_RGB", _KC_USE_STRM_RGB));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_USE_STRM_SKEL", _KC_USE_STRM_SKEL));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_USE_STRM_AUD", _KC_USE_STRM_AUD));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_USE_STRM_INTR", _KC_USE_STRM_INTR));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_DEPTH", _KC_DISP_DEPTH));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_RGB", _KC_DISP_RGB));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_AVATAR", _KC_DISP_AVATAR));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_MRKRS", _KC_DISP_MRKRS));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_AVTLOC", _KC_DISP_AVTLOC));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_RAWSK", _KC_DISP_RAWSK));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_FILTSK", _KC_DISP_FILTSK));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_SKCLIP", _KC_DISP_SKCLIP));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_UI_2D", _KC_DISP_UI_2D));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_UI_3D", _KC_DISP_UI_3D));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_DISP_UI_IK", _KC_DISP_UI_IK));	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_PROC_2D_DGRAB", _KC_PROC_2D_DGRAB)); 	
            KCidxVals.insert(std::pair<std::string, const int>("_KC_PROC_HANDS_2D", _KC_PROC_HANDS_2D));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_PROC_HANDS_3D", _KC_PROC_HANDS_3D));		
            KCidxVals.insert(std::pair<std::string, const int>("_KC_PROC_IK", _KC_PROC_IK));	
        }//initVars
    }//namespace kinect
}//namespace rtql8

