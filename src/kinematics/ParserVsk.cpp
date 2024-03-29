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

#include "ParserVsk.h"

// Standard Library
#include <map>
#include <sstream>
using namespace std;

// TiCPP library
// http://code.google.com/p/ticpp/
#include "ticpp.h"

#include <Eigen/Dense>
using namespace Eigen;

// Local Files
#include "Skeleton.h"
#include "BodyNode.h"
#include "Joint.h"
#include "Marker.h"
#include "Dof.h"
#include "Transformation.h"
#include "TrfmTranslate.h"
#include "TrfmRotateExpmap.h"
#include "TrfmRotateEuler.h"
#include "Marker.h"
#include "Shape.h"
#include "ShapeEllipsoid.h"
#include "ShapeCube.h"
#include "utils/UtilsRotation.h"
#include "utils/UtilsCode.h"

#define SCALE_VSK 1.0e-3
Vector3d expShoulder(0,0,0);
double lenShoulder = 0;

using namespace rtql8::kinematics;


// Forward Declarations of helper functions
Vector3d adjustPos(const Vector3d& _pos);
VectorXd getDofVectorXd(Transformation* trfm);

// Parsing Helper Functions    
bool readJointFree(ticpp::Element* _je, Joint* _jt, Skeleton* _skel);
bool readJointBall(ticpp::Element* _je, Joint* _jt, Skeleton* _skel, Vector3d orient);
bool readJointHardySpicer(ticpp::Element* _je, Joint* _jt, Skeleton* _skel);
bool readJointHinge(ticpp::Element* _je, Joint* _jt, Skeleton* _skel);
bool readSegment(ticpp::Element*_segment, BodyNode* _parent, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _skel);
bool readMarker(ticpp::Element*_marker, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _skel);
bool readShape(ticpp::Element* _prim, map<string, double>& _paramsList, map<string, double>& _massList, map<string, int>& _segmentindex, Skeleton* _skel);
void autoGenerateShape(Skeleton* skel);
void autoGenerateShapeParent(Skeleton* skel);


int readVSKFile(const char* const filename, Skeleton* _skel){
    cout << "Entering Read VSK File" << endl;

    // Load xml and create Document
    ticpp::Document _stateFile(filename);
    try
    {
        _stateFile.LoadFile();
    }
    catch(ticpp::Exception e)
    {
        cout << "LoadFile Fails: " << e.what() << endl;
        return VSK_ERROR;
    }
    cout << "Load " << filename << " (xml) Successfully" << endl;

    // Load Kinematic Model which defines Parameters, Skeletons and Markers
    ticpp::Element* kinmodel = NULL;
    kinmodel = _stateFile.FirstChildElement( "KinematicModel" );
    if(!kinmodel) return VSK_ERROR;

    // Read parameters and fill paramsList
    map<string, double> paramsList;
    paramsList.clear();
    {
        ticpp::Element* params = 0;
        params = kinmodel->FirstChildElement( "Parameters" );
        if(!params) return VSK_ERROR;
        // read all params
        ticpp::Iterator< ticpp::Element > childparam("Parameter");
        for ( childparam = childparam.begin( params ); childparam != childparam.end(); childparam++ ){
            string pname = childparam->GetAttribute("NAME");
            double val = 0; 
            childparam->GetAttribute("VALUE", &val);
            paramsList[pname] = val;
        }
    }

    // Read skeleton and fill the Skeleton* and segmentindex
    map<string, int> segmentindex;
    segmentindex.clear();
    {
        ticpp::Element* skel= 0;
        skel = kinmodel->FirstChildElement( "Skeleton" );
        if(!skel) return VSK_ERROR;
        // read all segments
        ticpp::Iterator< ticpp::Element > childseg("Segment");
        for ( childseg = childseg.begin( skel ); childseg != childseg.end(); childseg++ ){
            if(!readSegment(childseg->ToElement(), NULL, paramsList,
                segmentindex, _skel))
            {
                return VSK_ERROR;
            }
        }
    }        

    // Read markers and add them to _skel
    {
        ticpp::Element* markerset = 0;
        markerset = kinmodel->FirstChildElement( "MarkerSet" );
        if(!markerset) return VSK_ERROR;
        // read all markers
        ticpp::Element* markers = 0;
        markers = markerset->FirstChildElement( "Markers" );
        if(!markers) return VSK_ERROR;
        ticpp::Iterator< ticpp::Element > childmarker("Marker");
        for ( childmarker = childmarker.begin( markers ); childmarker != childmarker.end(); childmarker++ ){
            if(!readMarker(childmarker->ToElement(), paramsList,
                segmentindex, _skel))
            {
                return VSK_ERROR;
            }
        }
    }

    // Read masses
    ticpp::Element* masses = 0;
    map<string, double> masslist;
    masslist.clear();
    {
        try {
            masses = kinmodel->FirstChildElement( "Masses" );
            if(!masses) return false;
            ticpp::Iterator< ticpp::Element > childmass("Mass");
            for ( childmass = childmass.begin( masses ); childmass != childmass.end(); childmass++ ){
                string mname = childmass->GetAttribute("NAME");
                double mi=0; childmass->GetAttribute("VALUE", &mi);
                masslist[mname] = mi;
                cout <<"mass: "<<mname<<" "<<mi<<endl;
            }
        } 
        catch( ticpp::Exception ){
            cout <<"no masses found!\n";
        }
    }

    // Read primitives and fill the _skel
    {
        ticpp::Element* prims = 0;
        try {
            prims = kinmodel->FirstChildElement( "Shapes" );
            if(!prims) return false;
            ticpp::Iterator< ticpp::Element > childprim("Shape");
            for ( childprim = childprim.begin( prims ); childprim != childprim.end(); childprim++ ){
                if(!readShape(childprim->ToElement(), paramsList, masslist, segmentindex, _skel)) return false;
            }
        } 
        catch( ticpp::Exception ){}

        // fill in the default if prims absent
    }

    autoGenerateShapeParent(_skel);

    _skel->initSkel();
    // 
    cout << "VSK Parser exiting successfully" << endl;
    return VSK_OK;
}



bool readSegment(ticpp::Element*_segment, BodyNode* _parent, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _skel) {
    string sname = _segment->GetAttribute("NAME");

    cout <<"\nsegment: "<<sname<<" ";
    if(_parent) cout <<"parent: "<<_parent->getName()<<endl;
    else cout <<"parent: NULL\n";

    // make bodylink out of current segment
    BodyNode* blink = _skel->createBodyNode( sname.c_str() );

    // make a joint
    Joint* jt = new Joint(_parent, blink); 
    Vector3d orientation(0,0,0);

    // HARDCODED: constant rotation for humerus and changed translation
    if(sname.compare(1, 8, "humerus")!=0)
    {
        cout << "THE COMMON CODE!!" << endl;
        string txyz = _segment->GetAttribute("POSITION");
        vector<string> tokens; rtql8::utils::tokenize(txyz, tokens);
        assert(tokens.size()==3);
        Vector3d pos(0,0,0);
        for(unsigned int i=0; i<tokens.size(); i++) {
            string strval = tokens[i];
            int neg = 1;
            if(strval.c_str()[0]=='-') {
                neg = -1;
                strval.erase(strval.begin());
            }
            map<string, double>::iterator it = _paramsList.find(strval);
            if(it !=_paramsList.end()) pos[i] = neg*it->second;
            else {
                istringstream instr(tokens[i]);
                instr >> pos[i];
            }
        }
        Vector3d pos2 = adjustPos(pos);
        if(pos2 != Vector3d(0,0,0)){
            // create new transformation	
            Dof** dofs = new Dof*[3];
            for(int i=0; i<3; i++) {
                stringstream dofNameBuf;
                dofNameBuf << sname << "_" << i;
                string dofName = dofNameBuf.str();
                const char* pDofName = dofName.c_str();
                dofs[i] = new Dof(pos2[i], pDofName);
            }
            TrfmTranslate* tele = new TrfmTranslate(dofs[0],dofs[1],dofs[2]);
            // add transformation to joint
            jt->addTransform(tele, false);	
            // don't add to model because it's not variable
            cout<<"telescope: "<<pos2<<endl;
        }

        {
            string txtOrientation = _segment->GetAttribute("ORIENTATION");
            vector<string> tokens; rtql8::utils::tokenize(txtOrientation, tokens);
            assert(tokens.size()==3);
            for (int i = 0; i < 3; ++i) {
                orientation[i] = rtql8::utils::strTodouble(tokens[i]);
                // cout << "ORI = " << orientation[i] << " <== " << tokens[i] << endl;
            }
            orientation = adjustPos(orientation) / SCALE_VSK;
        }

    }
    // HARDCODED: constant rotation for humerus and changed translation
    else {
        cout << "HUMERUS SPECIAL CODE!!!!!" << endl;
        char lr = sname[0];
        //adjusted: +-Shoulder ShoulderHeight 0 
        string paramShoulder = "ShoulderLen";
        paramShoulder.push_back(lr);

        // create new transformations
        // telescope
        Vector3d pos(0.0, 1.0, 0.0);	// adjusted for skel
        pos *= _paramsList[paramShoulder];
        cout<<"shoulder len: "<<_paramsList[paramShoulder]<<endl;
        Dof** dofs = new Dof*[3];
        for(int i=0; i<3; i++) dofs[i] = new Dof(pos[i]);
        TrfmTranslate* tele = new TrfmTranslate(dofs[0],dofs[1],dofs[2]);
        // add transformation to joint
        jt->addTransform(tele, false);	
        // don't add to model because it's not variable
        cout<<"telescope: "<<pos<<endl;

        // const rotation
        Dof *dofx = new Dof(-expShoulder[0]);
        Dof *dofy = new Dof(-expShoulder[1]);
        Dof *dofz = new Dof(-expShoulder[2]);
        TrfmRotateExpMap *consrot= new TrfmRotateExpMap(dofx, dofy, dofz);
        jt->addTransform(consrot, false);	
        // don't add to model because it's not variable
        cout<<"const rotation: "<<-expShoulder<<endl;
    }

    // HARDCODED: constant rotation for clavicle
    if(sname.compare(1, 8, "clavicle")==0){
        cout << "CLAVICLE SPECIAL CODE!!" << endl;
        char lr = sname[0];
        string hname = "humerus";
        hname.insert(hname.begin(), lr);
        cout<<hname<<endl;
        // read the childsegment humerus
        ticpp::Element *humerus = _segment->FirstChildElement( "Segment");
        string cname = humerus->GetAttribute("NAME");
        if(cname.compare(hname)!=0){
            cout<<"Error: childname of "<<sname<<" doesnt match: "<<hname<<" vs "<<cname<<endl;
            return false;
        }
        // add telescope and const rot transforms
        string hxyz = humerus->GetAttribute("POSITION");
        vector<string> tokens; rtql8::utils::tokenize(hxyz, tokens);
        assert(tokens.size()==3);
        Vector3d pos(0,0,0);
        for(unsigned int i=0; i<tokens.size(); i++) {
            string strval = tokens[i];
            int neg = 1;
            if(strval.c_str()[0]=='-') {
                neg = -1;
                strval.erase(strval.begin());
            }
            map<string, double>::iterator it = _paramsList.find(strval);
            if(it !=_paramsList.end()) pos[i] = neg*it->second;
            else {
                istringstream instr(tokens[i]);
                instr >> pos[i];
            }
        }
        pos = adjustPos(pos);
        lenShoulder = pos.norm();
        string paramShoulder = "ShoulderLen";
        paramShoulder.push_back(lr);
        _paramsList[paramShoulder] = lenShoulder;
        cout<<"shoulder len: "<<_paramsList[paramShoulder]<<endl;
        //adjusted: +-Shoulder ShoulderHeight z 

        pos.normalize();
        double angleShoulder = pos[1];
        Vector3d axisShoulder = Vector3d(pos[2], 0, -pos[0]);
        axisShoulder.normalize();
        expShoulder = axisShoulder*angleShoulder;

        // create new transformation	
        Dof *dofx = new Dof(expShoulder[0]);
        Dof *dofy = new Dof(expShoulder[1]);
        Dof *dofz = new Dof(expShoulder[2]);
        TrfmRotateExpMap *consrot= new TrfmRotateExpMap(dofx, dofy, dofz);
        jt->addTransform(consrot, false);	
        // don't add to model because it's not variable
        cout<<"const rotation: "<<expShoulder<<endl;
    }


    // make transforms
    ticpp::Element *tf = 0;
    bool foundJoint = false;
    if(!foundJoint){
        try {
            tf = _segment->FirstChildElement( "JointFree" );
            foundJoint = true;
            readJointFree(tf, jt, _skel);
        }catch( ticpp::Exception e){
        }
    }
    if(!foundJoint){
        try {
            tf = _segment->FirstChildElement( "JointBall" );
            foundJoint = true;
            readJointBall(tf, jt, _skel, orientation);
        }catch( ticpp::Exception e){
        }
    }
    if(!foundJoint){
        try {
            tf = _segment->FirstChildElement( "JointHardySpicer" );
            foundJoint = true;
            readJointHardySpicer(tf, jt, _skel);
        }catch( ticpp::Exception e){
        }
    }
    if(!foundJoint){
        try {
            tf = _segment->FirstChildElement( "JointHinge" );
            foundJoint = true;
            readJointHinge(tf, jt, _skel);
        }catch( ticpp::Exception e){
        }
    }
    if(!foundJoint) cout<<"fixed joint!\n";

    for(int i=0; i<jt->getNumTransforms(); i++){
        if(!jt->getTransform(i)->getVariable()) continue;
        for(int j=0; j<jt->getTransform(i)->getNumDofs(); j++){
            cout<<jt->getTransform(i)->getDof(j)->getName()<<" ";
        }
        cout<<endl;
    }

    // add to the model
    _skel->addNode(blink);
    // _segmentindex[sname]=blink->getModelID();
    _segmentindex[sname]=blink->getSkelIndex();

    // marker the subtree
    ticpp::Iterator< ticpp::Element > childseg("Segment");
    for ( childseg = childseg.begin( _segment ); childseg != childseg.end(); childseg++ ){
        if(!readSegment(childseg->ToElement(), blink, _paramsList, _segmentindex, _skel)) return false;
    }
    return true;
}

bool readJointFree(ticpp::Element* _je, Joint* _jt, Skeleton* _skel) {
    cout<<"read free\n";

    // create new transformation
    string tname1 = string(_jt->getChildNode()->getName()) + "_t";
    string tname1_0 = tname1 + "Free0";
    string tname1_1 = tname1 + "Free1";
    string tname1_2 = tname1 + "Free2";
    vector<Dof*> dofs1;
    dofs1.resize(3);
    dofs1[0] = new Dof(0.0, tname1_0.c_str(), -100.0, 100.0);
    dofs1[1] = new Dof(0.0, tname1_1.c_str(), -100.0, 100.0);
    dofs1[2] = new Dof(0.0, tname1_2.c_str(), -100.0, 100.0);
    // dofs1[1] = new Dof(0.0, -100.0, 100.0);
    // dofs1[2] = new Dof(0.0, -100.0, 100.0);
    // add transformation to joint
    TrfmTranslate* trans = new TrfmTranslate(dofs1[0], dofs1[1], dofs1[2], tname1.c_str()); 
    _jt->addTransform(trans);	
    // add transformation to model because it's a variable dof
    _skel->addTransform(trans);

    string tname2 = string(_jt->getChildNode()->getName()) + "_a";
    string tname2_0 = tname2 + "Free3";
    string tname2_1 = tname2 + "Free4";
    string tname2_2 = tname2 + "Free5";
    vector<Dof*> dofs2;
    dofs2.resize(3);
    dofs2[0] = new Dof(0.0, tname2_0.c_str(), -3.1415, 3.1415);
    dofs2[1] = new Dof(0.0, tname2_1.c_str(), -3.1415, 3.1415);
    dofs2[2] = new Dof(0.0, tname2_2.c_str(), -3.1415, 3.1415);
    // dofs2[0] = new Dof(0.0, -3.1415, 3.1415);
    // dofs2[1] = new Dof(0.0, -3.1415, 3.1415);
    // dofs2[2] = new Dof(0.0, -3.1415, 3.1415);
    // add transformation to joint
    TrfmRotateExpMap* expmap= new TrfmRotateExpMap(dofs2[0], dofs2[1], dofs2[2], tname2.c_str());
    _jt->addTransform(expmap);	
    // add transformation to model because it's a variable dof
    _skel->addTransform(expmap);

    return true;
}

bool readJointBall(ticpp::Element* _je, Joint* _jt, Skeleton* _skel, Vector3d orient) {
    cout << "read ball\n";
    cout << "orientation = " << orient << endl;
    string tname2 = string(_jt->getChildNode()->getName()) + "_a";
    string tname2_0 = tname2 + "Ball0";
    string tname2_1 = tname2 + "Ball1";
    string tname2_2 = tname2 + "Ball2";
    vector<Dof*> dofs2;
    dofs2.resize(3);
    dofs2[0] = new Dof(orient[0], tname2_0.c_str(), -3.1415, 3.1415);
    dofs2[1] = new Dof(orient[1], tname2_1.c_str(), -3.1415, 3.1415);
    dofs2[2] = new Dof(orient[2], tname2_2.c_str(), -3.1415, 3.1415);


    // dofs2[1] = new Dof(0.0, -3.1415, 3.1415);
    // dofs2[2] = new Dof(0.0, -3.1415, 3.1415);
    // add transformation to joint
    TrfmRotateExpMap* expmap= new TrfmRotateExpMap(dofs2[0], dofs2[1], dofs2[2], tname2.c_str());
    _jt->addTransform(expmap);	
    // add transformation to model because it's a variable dof
    _skel->addTransform(expmap);

    return true;
}


bool readJointHardySpicer(ticpp::Element* _je, Joint* _jt, Skeleton* _skel) {
    cout<<"read hardy spicer\n";

    // Read axisxyz and parse it into tokens
    string axisxyz = _je->GetAttribute("AXIS-PAIR");
    vector<string> tokens;
    tokens.clear();

    string tname2 = string(_jt->getChildNode()->getName()) + "_a";
    string tname2_1 = tname2 + "Hardy0";
    const char* pTname1 = tname2_1.c_str();
    string tname2_2 = tname2 + "Hardy1";
    const char* pTname2 = tname2_2.c_str();


    // Use utils::tokenize
    tokens.clear();
    rtql8::utils::tokenize(axisxyz, tokens);
    assert(tokens.size()==6);

    // Which axis do we have?
    Transformation *r1=NULL;
    if(tokens[1].compare("1")==0){
        r1 = new TrfmRotateEulerX(new Dof(0.0, pTname1, -3.1415, 3.1415));
    }
    else if(tokens[2].compare("1")==0){
        r1 = new TrfmRotateEulerY(new Dof(0.0, pTname1, -3.1415, 3.1415));
    }
    else if(tokens[0].compare("1")==0){
        r1 = new TrfmRotateEulerZ(new Dof(0.0, pTname1, -3.1415, 3.1415));
    }
    assert(r1!=NULL);
    _jt->addTransform(r1);	
    _skel->addTransform(r1);

    Transformation *r2=NULL;
    if(tokens[4].compare("1")==0){
        r2 = new TrfmRotateEulerX(new Dof(0.0, pTname2, -3.1415, 3.1415));
    }
    else if(tokens[5].compare("1")==0){
        r2 = new TrfmRotateEulerY(new Dof(0.0, pTname2, -3.1415, 3.1415));
    }
    else if(tokens[3].compare("1")==0){
        r2 = new TrfmRotateEulerZ(new Dof(0.0, pTname2, -3.1415, 3.1415));
    }
    assert(r2!=NULL);
    _jt->addTransform(r2);	
    _skel->addTransform(r2);

    return true;
}


bool readJointHinge(ticpp::Element* _je, Joint* _jt, Skeleton* _skel) {
    cout<<"read hinge\n";

    string tname = string(_jt->getChildNode()->getName()) + "_a";
    tname += "Hinge0";
    const char* pTname = tname.c_str();

    string axisxyz = _je->GetAttribute("AXIS");
    vector<string> tokens;
    tokens.clear();

    // Use utils::tokenize
    rtql8::utils::tokenize(axisxyz, tokens);
    assert(tokens.size()==3);

    // Read axes data
    Transformation *r1=NULL;
    Vector3d axis(rtql8::utils::strTodouble(tokens[0]),
        rtql8::utils::strTodouble(tokens[1]),
        rtql8::utils::strTodouble(tokens[2]));
    // if(tokens[1].compare("1")==0){
    if ((axis - adjustPos(Vector3d(1.0, 0.0, 0.0)) / SCALE_VSK ).norm() < 0.01) {
        r1 = new TrfmRotateEulerX(new Dof(0.0, pTname, -3.1415, 3.1415));
        cout << "RotateEulerX" << endl;
    }
    else if ((axis - adjustPos(Vector3d(0.0, 1.0, 0.0)) / SCALE_VSK ).norm() < 0.01) {
        // else if(tokens[2].compare("1")==0){
        r1 = new TrfmRotateEulerY(new Dof(0.0, pTname, -3.1415, 3.1415));
        cout << "RotateEulerY" << endl;
    }
    else if ((axis - adjustPos(Vector3d(0.0, 0.0, 1.0)) / SCALE_VSK ).norm() < 0.01) {
        // else if(tokens[0].compare("1")==0){
        r1 = new TrfmRotateEulerZ(new Dof(0.0, pTname, -3.1415, 3.1415));
        cout << "RotateEulerZ" << endl;
    }
    assert(r1!=NULL);
    _jt->addTransform(r1);	
    _skel->addTransform(r1);

    return true;
}

bool readMarker(ticpp::Element*_marker, map<string, double>& _paramsList, map<string, int>& _segmentindex, Skeleton* _skel) {
    string mname = _marker->GetAttribute("NAME");
    string sname = _marker->GetAttribute("SEGMENT");

    // get the local position
    string pxyz = _marker->GetAttribute("POSITION");
    vector<string> tokens;
    rtql8::utils::tokenize(pxyz, tokens);
    assert(tokens.size()==3);


    Vector3d lpos(0,0,0);
    for(unsigned int i=0; i<tokens.size(); i++) {
        string strval = tokens[i];
        int neg = 1;
        if(strval.c_str()[0]=='-') {
            neg = -1;
            strval.erase(strval.begin());
        }
        map<string, double>::iterator it = _paramsList.find(strval);
        if(it !=_paramsList.end()) lpos[i] = neg*it->second;
        else {
            istringstream instr(tokens[i]);
            instr >> lpos[i];
        }
    }
    // rearrange the coordinates
    Vector3d lpos2 = adjustPos(lpos);

    // HARDCODED for clavicle
    if(sname.compare(1, 8, "clavicle")==0){
        // char lr = sname[0];
        // compute angle for the clavicle
        // left first; so negate the previous computed for right in reading segments and then same for the right
        expShoulder = -expShoulder;	

        // create new  position
        Vector3d negExpShoulder = -expShoulder;
        Quaterniond qr = rtql8::utils::rotation::expToQuat(negExpShoulder);	// negative for the markers
        rtql8::utils::rotation::rotatePoint(qr, lpos2);
    }

    Marker* m = new Marker(mname.c_str(), lpos2, _skel->getNode(_segmentindex[sname]));
    _skel->addMarker(m);
    cout<<"marker: "<<mname<<" ";
    cout<<"segment: "<<sname<<" ";
    cout<<"sindex: "<<_segmentindex[sname]<<" ";
    cout<<"lpos: "<<lpos2<<endl;
    return true;
}


bool readShape(ticpp::Element* _prim, map<string, double>& _paramsList, map<string, double>& _massList, map<string, int>& _segmentindex, Skeleton* _skel) {
    string bname = _prim->GetAttribute("SEGMENT");
    int segIdx = _segmentindex[bname];
    BodyNode* blink = _skel->getNode(segIdx);

    string mname = _prim->GetAttribute("MASS");
    double mass = _massList[mname];

    string sname = _prim->GetAttribute("SCALE");
    double scale = 0;
    map<string, double>::iterator it = _paramsList.find(sname);
    if(it !=_paramsList.end()) scale = it->second;
    else {
        istringstream instr(sname);
        instr >> scale;
    }

    string dimxyz = _prim->GetAttribute("DIMENSION");
    vector<string> tokens;
    rtql8::utils::tokenize(dimxyz, tokens);

    assert(tokens.size()==3);

    Vector3d dim(0,0,0);
    for(unsigned int i=0; i<tokens.size(); i++) {
        istringstream instr(tokens[i]);
        instr >> dim[i];
    }
    dim = adjustPos(dim*scale);

    string offxyz = _prim->GetAttribute("OFFSET");
    rtql8::utils::tokenize(offxyz, tokens);
    assert(tokens.size()==3);

    Vector3d off(0,0,0);
    for(unsigned int i=0; i<tokens.size(); i++) {
        istringstream instr(tokens[i]);
        instr >> off[i];
    }
    off = adjustPos(off*scale);


    Shape *prim = NULL;
    string ptype = _prim->GetAttribute("TYPE");
    if(ptype.compare("ELLIPSOID")==0){
        prim = new ShapeEllipsoid(dim, mass);
    }
    // else if(ptype.compare("SPHERE")==0){
    //     prim = new ShapeSphere(off, dim[0], mass);
    // }
    // else if(ptype.compare("CYLINDER")==0){
    //     prim = new ShapeCylinder(off, dim, mass);
    // }
    // else if(ptype.compare("CYLINDERX")==0){
    //     prim = new ShapeCylinderX(off, dim, mass);
    // }
    // else if(ptype.compare("CYLINDERZ")==0){
    //     prim = new ShapeCylinderZ(off, dim, mass);
    // }
    // else if(ptype.compare("HEAD")==0){
    //     prim = new ShapeHead(off, dim, mass);
    // }
    else if(ptype.compare("CUBE")==0){
        prim = new ShapeCube(dim, mass);
    }
    else {
        cout << "Shape type " << ptype << " not recognized!\n";
        return false;
    }

    //set color	
    try {
        string cname = _prim->GetAttribute("RGB");
        tokens.clear();
        rtql8::utils::tokenize(cname, tokens);
        if (tokens.size() == 3)
        {
            Vector3d clr(0,0,0);
            for(unsigned int i=0; i<tokens.size(); i++) {
                istringstream instr(tokens[i]);
                instr >> clr[i];
            }
            prim->setColor(clr/255.0);
        }
        else
        {
            prim->setColor(Vector3d(0.5, 0.5, 1.0));
        }
    } 
    catch( ticpp::Exception ){
    }

    blink->setShape(prim);
    blink->setLocalCOM( off );
    return true;
}


Vector3d adjustPos(const Vector3d& _pos) {
    // rearrange the coordinates
    Vector3d pos2 =_pos;
    // pos2[0] = _pos[1];
    // pos2[1] = _pos[2];
    // pos2[2] = _pos[0];
    pos2 *= SCALE_VSK;
    return pos2;
}

VectorXd getDofVectorXd(Transformation* tr) {
    const int nDofs = tr->getNumDofs();
    VectorXd data(nDofs);
    for (int i = 0; i < nDofs; ++i) {
        Dof* dof = tr->getDof(i);
        data[i] = dof->getValue();
    }
    return data;
}

void autoGenerateShape(Skeleton* skel) {
    for(int i=0; i<skel->getNumNodes(); i++){
        if(skel->getNode(i)->getShape()) continue;
        ShapeEllipsoid *pm = new ShapeEllipsoid(0.05 * Vector3d(1.0,1.0,1.0), 1.0);
        pm->setColor(Vector3d(0.5, 0.5, 1.0));
        BodyNode* node = skel->getNode(i);
        node->setShape(pm);
        Vector3d vecZero(0,0,0);
        node->setLocalCOM(vecZero);
    }
}

void autoGenerateShapeParent(Skeleton* skel)
{
    // autoGenerateShape(skel); return;

    cout << "Auto-generating primitives" << endl;

    double massSum = 0.0;
    for(int i=0; i<skel->getNumNodes(); i++){
        BodyNode* node = skel->getNode(i);
        Joint* joint = node->getParentJoint();
        if(node->getShape()) continue;
        // Search translate matrix
        Vector3d size = 0.1 * Vector3d(1,1,1);
        Vector3d offset(0,0,0);
        cout << endl;
        cout << "Node = " << node->getName() << endl;
        if (node->getParentNode() == NULL)
        {
            cout << "computing size for the root" << endl;
            size = 0.1 * Vector3d(1,1,1);
            continue;
        }
        BodyNode* parent = node->getParentNode();

        cout << "Parent Node = " << node->getParentNode()->getName() << endl;
        for (int j = 0; j < joint->getNumTransforms(); ++j)
        {
            Transformation* trfm = joint->getTransform(j);
            if (trfm->getType() == Transformation::T_TRANSLATE)
            {
                const VectorXd dofdata = getDofVectorXd(trfm);
                if (dofdata.size() == 3)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        size[k] = fabs(dofdata[k]);
                        offset[k] = dofdata[k] * 0.5;
                    }
                    break;
                }
            }
        }

        double maxSize = max(size[0], size[1]);
        maxSize = max(maxSize, size[2]);
        maxSize *= 0.35;
        maxSize = min(0.1, maxSize);

        for (int j = 0; j < 3; ++j)
        {
            size[j] = max(size[j], maxSize);
        }

        double density = 2000.0;
        double mass = density * size[0] * size[1] * size[2];
        massSum += mass;
        cout << "Size = " << size << endl;
        cout << "Offset = " << offset << endl;
        cout << "Mass = " << mass << endl;

        // size = 0.1 * Vector3d(vl_1);
        // offset = Vector3d(vl_0);
        // Ellipsoid *pm = new Ellipsoid(vl_0, 0.1*Vector3d(vl_1), 1.0);
        ShapeEllipsoid *pm = new ShapeEllipsoid(size, mass);
        BodyNode* target = parent;
        target->setLocalCOM(offset);
        pm->setColor(Vector3d(0.5, 0.5, 1.0));
        target->setShape(pm);

    }

    autoGenerateShape(skel);
    cout << "Sum of mass = " << massSum << endl;
}

