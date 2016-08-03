/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "rig.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include "common/app_cppcommon.h"
#include "common/fullbody1.h"
#include "rapidxml_helper.h"
#include "app_controller_state.h"

namespace controller {
////////////////////////////////////////////////////////////
// class Rig implementation
    Rig::Rig()
        : MEMBER_INIT(upper, 1.0)
        , MEMBER_INIT(lower, -1.0)
        , MEMBER_INIT(hasDesiredParam, false)
        , MEMBER_INIT(desiredParam, 0.0)
    {
        setParams1D(0.0);
    }
    
    Rig::~Rig() {
    }

// Auxiliary functions
    std::string Rig::toString() const {
        std::string str;
        str += "[" + name + " ";
        str += rtql8::toolkit::moreeigen::convertVectorXdToString(params);
        if (hasDesiredParam()) {
            // str += " (" + boost::lexical_cast<std::string>(desiredParam()) + ")";
            str += " (" + (boost::format("%.4lf") % desiredParam()).str() + ")";
        }
        str += "]";
        return str;
    }

// XML functions
    Rig* Rig::readXML(rapidxml::xml_node<char>* node) {
        // std::string tag = rapidxml::tag(node);
        std::string name = rapidxml::attr(node, "name");
        Rig* rig = NULL;
        if (boost::starts_with(name, "legpose")) {
            rig = (new RigLegPose())->readXML(node);
        } else if (boost::starts_with(name, "armpose")) {
            rig = (new RigArmPose())->readXML(node);
        } else if (boost::starts_with(name, "joints")) {
            rig = (new RigJoints())->readXML(node);
        } else if (boost::starts_with(name, "feedbackjoints")) {
            rig = (new RigFeedbackJoints())->readXML(node);
        } else if (boost::starts_with(name, "vf")) {
            rig = (new RigVF())->readXML(node);
        } else if (boost::starts_with(name, "stiffjoints")) {
            rig = (new RigStiffJoints())->readXML(node);
        } else {
            LOG_FATAL << "invalid rig. name = " << name;
        }
        return rig;
    }

    rapidxml::xml_node<char>* Rig::writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "controller");
        rx::add_attr(pdoc, node, "name", name.c_str());
        return node;
    }

    void Rig::setDesiredParam(double _p) {
        set_hasDesiredParam(true);
        set_desiredParam(_p);
    }

    double Rig::cost() {
        if (hasDesiredParam()) {
            double d = params(0) - desiredParam();
            return 0.5 * d * d;
        }
        return 0.0;
    }

// class Rig ends
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
// class RigJoints implementation
    RigJoints::RigJoints() {
    }
    
    RigJoints::~RigJoints() {
    }

    void RigJoints::changePose() {
        using namespace fullbody1;
        double p = params(0);
        Eigen::VectorXd qhat = target();
        double w = 3.14;
        // double w = 1.0;
        double theta = w * p;

        // cout << endl;
        // cout << "prev = " << IO(qhat) << endl;
        if (axis() == "heels") {
            qhat(l_ankle_ez)  += theta;
            qhat(r_ankle_ez)  += theta;
        } else if (axis() == "hips") {
            qhat(l_thigh_ez)  += theta;
            qhat(r_thigh_ez)  += theta;
        } else if (axis() == "knees") {
            qhat(l_knee_ez)  += theta;
            qhat(r_knee_ez)  += theta;
        } else if (axis() == "spine") {
            qhat(abdomen_ez) += theta;
            qhat(neck_ez)    += theta;
        } else {
            LOG_FATAL << "invalid joint set: " << axis();
            exit(0);
        }
        // cout << "curr = " << IO(qhat) << endl;
        // cout << endl;

        setTarget(qhat);
    }

// XML functions
    RigJoints* RigJoints::readXML(rapidxml::xml_node<char>* node) {
        namespace rx = rapidxml;
        std::string n = rapidxml::attr(node, "name");
        std::string stem = "joints";
        set_axis( n.substr( stem.length() + 1) );
        // this->name = "Joints_" + axis();
        this->name = n;

        return this;
    }
// class RigJoints ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RigFeedbackJoints implementation
    RigFeedbackJoints::RigFeedbackJoints() {
    }
    
    RigFeedbackJoints::~RigFeedbackJoints() {
    }

    void RigFeedbackJoints::exertForce() { // Sounds hacky, but for simplicity
        using namespace fullbody1;
        double p = params(0);
        Eigen::VectorXd qhat = target();
        double w = 10.0;
        Eigen::Vector3d C = COM();
        // Eigen::Vector3d P = COP();
        Eigen::Vector3d P = APOS(4, l_heel, r_heel, l_toe, r_toe);
        double d = C.x() - P.x();
        double v = COMdot().x();
        double theta = p * (w * d);
        // double theta_v = 0.5 * sqrt(fabs(p * w)) * v;
        // if (p < 0.0) {
        //     theta_v = -theta_v;
        // }
        // theta += theta_v;
        // cout << "p * w = " << p * w << endl;
        // cout << "sqrt(p * w) = " << sqrt(fabs(p * w)) << endl;
        
        // theta = 0.5;

        if (axis() == "heels") {
            qhat(l_ankle_ez)  += theta;
            qhat(r_ankle_ez)  += theta;
        } else if (axis() == "hips") {
            qhat(l_thigh_ez)  += theta;
            qhat(r_thigh_ez)  += theta;
        } else if (axis() == "knees") {
            qhat(l_knee_ez)  += theta;
            qhat(r_knee_ez)  += theta;
        }


        // cout << "d = " << d << " ";
        // cout << "v = " << v << " ";
        // cout << "theta = " << theta << " ";
        // cout << "C = " << IO(C) << " ";
        // cout << "P = " << IO(P) << " ";
        // cout << "COMdot =" << IO(COMdot()) << " ";
        // cout << endl;

        adjustTarget(qhat);
    }

// XML functions
    RigFeedbackJoints* RigFeedbackJoints::readXML(rapidxml::xml_node<char>* node) {
        namespace rx = rapidxml;
        std::string n = rapidxml::attr(node, "name");
        std::string stem = "feedbackjoints";
        set_axis( n.substr( stem.length() + 1) );
        // this->name = "FeedbackJoints_" + axis();
        this->name = n;
        return this;
    }
// class RigFeedbackJoints ends
////////////////////////////////////////////////////////////

    
////////////////////////////////////////////////////////////
// class RigLegPose implementation
    RigLegPose::RigLegPose() {
    }
    
    RigLegPose::~RigLegPose() {
    }

    void RigLegPose::changePose() {
        using namespace fullbody1;
        double p = params(0);
        Eigen::VectorXd qhat = target();
        // cout << endl;
        // cout << "prev = " << IO(qhat) << endl;
        if (axis() == "d") {
            double w0 = -1.0;
            double w1 = 2.0;
            double w2 = -1.0;
            qhat(l_thigh_ez) += w0 * p;
            qhat(r_thigh_ez) += w0 * p;
            qhat(l_knee_ez)  += w1 * p;
            qhat(r_knee_ez)  += w1 * p;
            qhat(l_ankle_ez) += w2 * p;
            qhat(r_ankle_ez) += w2 * p;
        } else if (axis() == "a") {
            double w0 = 2.0;
            qhat(l_thigh_ez) += w0 * p;
            qhat(r_thigh_ez) += w0 * p;
        } else if (axis() == "s") {
            double w0 = 2.0;
            qhat(l_thigh_ez) += w0 * p;
            qhat(r_thigh_ez) -= w0 * p;
        } else {
        }
        // cout << "curr = " << IO(qhat) << endl;
        // cout << endl;
        setTarget(qhat);
    }

// XML functions
    RigLegPose* RigLegPose::readXML(rapidxml::xml_node<char>* node) {
        namespace rx = rapidxml;
        std::string n = rapidxml::attr(node, "name");
        set_axis( n.substr( n.length() - 1) );
        // this->name = "LegPose_" + axis();
        this->name = n;

        return this;
    }

// class RigLegPose ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RigArmPose implementation
    RigArmPose::RigArmPose() {
    }
    
    RigArmPose::~RigArmPose() {
    }

    void RigArmPose::changePose() {
        using namespace fullbody1;
        double p = params(0);
        Eigen::VectorXd qhat = target();
        // cout << endl;
        // cout << "prev = " << IO(qhat) << endl;
        if (axis() == "a") {
            double w0 = -3.0;
            double w1 = 1.0;
            qhat(l_bicep_ey) += w0 * p;
            qhat(l_bicep_ex) += w1 * p;

            qhat(r_bicep_ey) -= w0 * p;
            qhat(r_bicep_ex) -= w1 * p;
        } else if (axis() == "b") {
            double w0 = -3.0;
            double w1 = 0.0;
            qhat(l_bicep_ey) += w0 * p;
            qhat(l_bicep_ex) += w1 * p;

            qhat(r_bicep_ey) -= w0 * p;
            qhat(r_bicep_ex) -= w1 * p;
        } else if (axis() == "d") {
            double w0 = 3.0;
            qhat(l_elbow_ez) += w0 * p;
            qhat(r_elbow_ez) += w0 * p;

        } 
        // cout << "curr = " << IO(qhat) << endl;
        // cout << endl;
        setTarget(qhat);
    }

// XML functions
    RigArmPose* RigArmPose::readXML(rapidxml::xml_node<char>* node) {
        namespace rx = rapidxml;
        std::string n = rapidxml::attr(node, "name");
        set_axis( n.substr( n.length() - 1) );
        // this->name = "ArmPose_" + axis();
        this->name = n;

        return this;
    }

// class RigArmPose ends
////////////////////////////////////////////////////////////

    
////////////////////////////////////////////////////////////
// class RigVF implementation
    RigVF::RigVF() {
    }
    
    RigVF::~RigVF() {
    }

    void RigVF::exertForce() {
        using namespace fullbody1;
        double p = params(0);

        Eigen::Vector3d C = COM();
        Eigen::Vector3d P = APOS(2, l_toe, r_toe);
        Eigen::Vector3d CP = C - P;
        CP /= CP.norm();

        Eigen::Vector3d Y = CP;
        // Eigen::Vector3d Y(0, 1, 0);
        Eigen::Vector3d Z = Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d X = CP.cross(Z);
        X *= 0.5;
        // cout << "Y = " << IO(Y) << endl;
        // cout << "X = " << IO(X) << endl;

        const double MAX_F = 3000.0;
        AppControllerState::VF vf1(l_toe, MAX_F * p, MAX_F * p, MAX_F * p);
        AppControllerState::VF vf2(r_toe, MAX_F * p, MAX_F * p, MAX_F * p);
        if (axis() == "y") {
            // vf1.f = vf1.f.cwiseProduct(Eigen::Vector3d(0.0, 1.0, 0.0));
            // vf2.f = vf2.f.cwiseProduct(Eigen::Vector3d(0.0, 1.0, 0.0));
            vf1.f = vf1.f.cwiseProduct(Y);
            vf2.f = vf2.f.cwiseProduct(Y);
        } else if (axis() == "x") {
            vf1.f = vf1.f.cwiseProduct(X);
            vf2.f = vf2.f.cwiseProduct(X);
        }

        vf1.p = Eigen::Vector3d(0.02, 0.0, 0.0);
        vf2.p = Eigen::Vector3d(0.02, 0.0, 0.0);
                
        conState()->vfs.push_back(vf1);
        conState()->vfs.push_back(vf2);

        // if (axis() == "y") {
        // } else if (axis() == "x") {
        //     const double MAX_F = 3000.0;
        //     const double MAX_F = 3000.0;
        //     AppControllerState::VF vf1(l_toe, 0, MAX_F * p, 0);
        //     AppControllerState::VF vf2(r_toe, 0, MAX_F * p, 0);
        //     vf1.p = Eigen::VectorXd(0, 0.20, 0);
        //     vf2.p = Eigen::VectorXd(0, 0.20, 0);
                
        //     conState()->vfs.push_back(vf1);
        //     conState()->vfs.push_back(vf2);

        //     conState()->vfs.push_back(AppControllerState::VF(l_toe, MAX_F * p, 0, 0));
        //     conState()->vfs.push_back(AppControllerState::VF(r_toe, MAX_F * p, 0, 0));
        // }
    }
    
// XML functions
    RigVF* RigVF::readXML(rapidxml::xml_node<char>* node) {
        namespace rx = rapidxml;
        std::string n = rapidxml::attr(node, "name");
        set_axis( n.substr( n.length() - 1) );
        // this->name = "VF_" + axis();
        this->name = n;
                                     
        return this;
    }

    // rapidxml::xml_node<char>* RigVF::writeXML(rapidxml::xml_document<char>* pdoc) {
    //     namespace rx = rapidxml;
    //     rx::xml_node<char>* node = rx::alloc_node(pdoc, "vf");
    //     rx::add_attr(pdoc, node, "axis", axis().c_str());
    //     rx::add_attr(pdoc, node, "params", params);
    //     return node;
    // }

// class RigVF ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RigStiffJoints implementation
    RigStiffJoints::RigStiffJoints() {
    }
    
    RigStiffJoints::~RigStiffJoints() {
    }

    void RigStiffJoints::changePose() {
        using namespace fullbody1;
        double p = params(0);
        Eigen::VectorXd kp = stiffness();
        double s= 1.0;
        if (p < 0.0) {
            s = exp( 6.0 * p );
        } else {
            s = exp( 2.0 * p );
        }

        // cout << endl;
        // cout << "prev = " << IO(qhat) << endl;
        if (axis() == "shoulders") {
            kp(l_bicep_ey)  *= s;
            kp(l_bicep_ex)  *= s;
            kp(r_bicep_ey)  *= s;
            kp(r_bicep_ex)  *= s;
            kp(l_elbow_ez)  *= s;
            kp(r_elbow_ez)  *= s;
        } else if (axis() == "hips") {
            kp(l_thigh_ez)  *= s;
            kp(r_thigh_ez)  *= s;
        } else {
            LOG_FATAL << "invalid joint set: " << axis();
            exit(0);
        }

        setStiffness(kp);
    }

// XML functions
    RigStiffJoints* RigStiffJoints::readXML(rapidxml::xml_node<char>* node) {
        namespace rx = rapidxml;
        std::string n = rapidxml::attr(node, "name");
        std::string stem = "stiffjoints";
        set_axis( n.substr( stem.length() + 1) );
        // this->name = "Joints_" + axis();
        this->name = n;

        return this;
    }
// class RigStiffJoints ends
////////////////////////////////////////////////////////////
    
    
} // namespace controller



