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

#ifndef __KINCNTRLRXMLREADER_H__
#define __KINCNTRLRXMLREADER_H__

using namespace std;
#include <vector>
#include "KinectController.h"


//this class provides an interface to populate user-modifiable quantities to layout a ui for the kinect to interact with
namespace rtql8{
    namespace kinect{
        class KinectController;
        class KinCntrlrXMLReader {
        public:
            KinCntrlrXMLReader(){}
            KinCntrlrXMLReader(KinectController* _kc, bool _autoUpd=true);
            virtual ~KinCntrlrXMLReader();
            void initVars();

            bool readFile(const char* const filename);
            void readDisplayStyle(ticpp::Element& node);                        //style sheet values for all UI objects

            //need to be read first, for all projects, from default xml file
            void readConstants(ticpp::Element& node);  
            void readStateControl(ticpp::Element& node);  
            void updateHandlerStateFlags();

            KinUIPage* readUIPage(ticpp::Element& node);
            KinUIButton* readUIButton(ticpp::Element& node);
            KinUISlider* readUISlider(ticpp::Element& node);
            KinUICrank* readUICrank(ticpp::Element& node);
            KinUIImageButton* readUIImgButton(ticpp::Element& node);
            KinUIPrgBar* readUIPrgBar(ticpp::Element& node);
            KinUITimer* readUITimer(ticpp::Element& node);
            KinUITextBox* readUITextBox(ticpp::Element& node);
            KinUICaption* readUICaption(ticpp::Element& node);

            void readBaseCmpAttr(ticpp::Element& node, KinUIComponent* cmp);
            void readBaseSliderAttr(ticpp::Element& node, KinUISlider* cmp);
            void readBaseCmpColor(ticpp::Element& node, KinUIComponent* cmp);
            void readClassAttrAndColor(ticpp::Element& node, KinUIComponent* cmp);
            void readClassAttrAndColor(const char* const name, KinUIComponent* cmp);


        protected :	//methods


        protected : //vars
            KinectController* kc;
            map<std::string, const int> KCidxVals;                              //map matching string names of idx's to the program constants they represent	
            vector<bool> hndlrFlageChg;                                         //whether or not a particular handler has had flag values change
            bool autoUpd;                                                       //whether or not the flags can be updated immediately after they have been read in from file - disable for default xml load only
            map<int, bool>tmpKH_flags;                                          //kinect handler flags
            map<int, bool>tmpAH_flags;                                          //audio handler flags
            map<int, bool>tmpDH_flags;                                          //depth handler flags
            map<int, bool>tmpIMH_flags;                                         //image handler flags
            map<int, bool>tmpINH_flags;                                         //interaction handler flags
            map<int, bool>tmpSKH_flags;                                         //skel handler flags    
            map<int, bool>tmpKC_flags;                                          //kinect controller flags
            map<int, bool>tmpKUI_flags;                                         //default ui object flags - overridden when individual objects are made

            std::vector<ticpp::Element*> classElements;

        };//KinCntrlrXMLReader class
    }//namespace kinect
}//namespace rtql8
#endif
