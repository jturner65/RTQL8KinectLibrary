/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "interpreter.h"
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "common/app_cppcommon.h"
#include "common/fullbody1.h"

#include "controller/knowledge.h"
#include "controller/simpack.h"
#include "op_controller.h"


namespace operation {
    
////////////////////////////////////////////////////////////
// class Interpreter implementation
    Interpreter::Interpreter(controller::Knowledge* _kn)
        : MEMBER_INIT(kn, _kn)
    {
    }
    
    Interpreter::~Interpreter() {
    }

    bool Interpreter::parse(const char* const instruction,
                            bool* pRunopt, bool* pResetMotions) {
        (*pRunopt) = false;
        std::vector<std::string> tokens;
        boost::split(tokens, instruction, boost::is_any_of("\t "));
        while(tokens.size() > 0 && tokens[tokens.size() - 1].length() == 0) {
            tokens.pop_back();
        }
        for (int i = 0; i < tokens.size(); i++) {
            LOG_INFO << "token " << i << " : " << tokens[i]
                     << "(" << tokens[i].length() << ")";
        }

        int n = tokens.size();
        if (n < 1) {
            LOG_WARNING << "command not detected";
            return false;
        }
        std::string cmd = tokens[0];
        if (cmd == "addrig" || cmd == "using") {
            OpController op(kn());
            op.addRig(tokens[1].c_str());
            (*pRunopt) = true;
        } else if (cmd == "addprev" || cmd == "startfrom") {
            OpController op(kn());
            op.addPrev(tokens[1].c_str());
            (*pRunopt) = true;
        } else if (cmd == "addcost" || cmd == "consider") {
            OpController op(kn());
            if (tokens[1] == "landing") {
                op.addCostLanding();
            } else if (tokens[1] == "upright") {
                (*pRunopt) = true;
                double a0 = boost::lexical_cast<double>(tokens[2]);
                double a1 = boost::lexical_cast<double>(tokens[4]);
                op.addCostUpright2(a0, a1);
            }
        } else if (cmd == "addrigas") {
            OpController op(kn());
            double p = boost::lexical_cast<double>(tokens[2]);
            op.addRig(tokens[1].c_str(), p);
            (*pRunopt) = true;
        } else if (cmd == "movedown") {
            CHECK_EQ(n, 4) << "Invalid syntax: movedown <v0> to <v1>";
            OpController op(kn());
            double v0 = boost::lexical_cast<double>(tokens[1]);
            double v1 = boost::lexical_cast<double>(tokens[3]);
            Eigen::Vector3d vv0;
            Eigen::Vector3d vv1;
            op.findCOMTerm(&vv0, &vv1);
            LOG_INFO << "Detected = " << IO(vv0) << " " << IO(vv1);
            vv0(1) = v0;
            vv1(1) = v1;
            op.move(vv0, vv1);
            (*pRunopt) = true;
        } else if (cmd == "moveforward") {
            CHECK_EQ(n, 4) << "Invalid syntax: moveforward <v0> to <v1>";
            OpController op(kn());
            double v0 = boost::lexical_cast<double>(tokens[1]);
            double v1 = boost::lexical_cast<double>(tokens[3]);
            Eigen::Vector3d vv0;
            Eigen::Vector3d vv1;
            op.findCOMTerm(&vv0, &vv1);
            LOG_INFO << "Detected = " << IO(vv0) << " " << IO(vv1);
            vv0(0) = v0;
            vv1(0) = v1;
            op.move(vv0, vv1);
            (*pRunopt) = true;
        } else if (cmd == "movemin") {
            CHECK_EQ(n, 4) << "Invalid syntax: movemin <v0> to <v1>";
            OpController op(kn());
            double v0 = boost::lexical_cast<double>(tokens[1]);
            double v1 = boost::lexical_cast<double>(tokens[3]);
            op.moveMin(Eigen::Vector3d(0, v0, 0), Eigen::Vector3d(0, v1, 0));
            (*pRunopt) = true;
        } else if (cmd == "movemax") {
            CHECK_EQ(n, 4) << "Invalid syntax: movemax <v0> to <v1>";
            OpController op(kn());
            double v0 = boost::lexical_cast<double>(tokens[1]);
            double v1 = boost::lexical_cast<double>(tokens[3]);
            op.moveMax(Eigen::Vector3d(0, v0, 0), Eigen::Vector3d(0, v1, 0));
            (*pRunopt) = true;
        } else if (cmd == "speed") {
            CHECK_EQ(n, 4) << "Invalid syntax: speed <v0> to <v1>";
            OpController op(kn());
            double v0 = boost::lexical_cast<double>(tokens[1]);
            double v1 = boost::lexical_cast<double>(tokens[3]);
            Eigen::Vector3d vv0;
            Eigen::Vector3d vv1;
            op.findCOMTerm(&vv0, &vv1);
            LOG_INFO << "COMDot.Detected = " << IO(vv0) << " " << IO(vv1);
            vv0(1) = v0;
            vv1(1) = v1;
            op.speed(vv0, vv1);
            (*pRunopt) = true;
        } else if (cmd == "speedhori") {
            CHECK_EQ(n, 4) << "Invalid syntax: speedhori <v0> to <v1>";
            OpController op(kn());
            double v0 = boost::lexical_cast<double>(tokens[1]);
            double v1 = boost::lexical_cast<double>(tokens[3]);
            Eigen::Vector3d vv0;
            Eigen::Vector3d vv1;
            op.findCOMTerm(&vv0, &vv1);
            LOG_INFO << "COMDot.Detected = " << IO(vv0) << " " << IO(vv1);
            vv0(0) = v0;
            vv1(0) = v1;
            op.speed(vv0, vv1);
            (*pRunopt) = true;
        } else if (cmd == "spin") {
            CHECK_EQ(n, 2) << "Invalid syntax: spin <v0>";
            OpController op(kn());
            double v0 = boost::lexical_cast<double>(tokens[1]);
            op.spin(Eigen::Vector3d(0, 0, v0));
            (*pRunopt) = true;
        } else if (cmd == "initialpose" || cmd == "init" ) {
            CHECK_EQ(n, 2) << "Invalid syntax: initialpose <posename>";
            OpController op(kn());
            Eigen::VectorXd pose = parsePose(tokens[1].c_str());
            op.changeInitialPose(pose);
        } else if (cmd == "terminate") {
            OpController op(kn());
            if (tokens[1] == "timeout") {
                double t = boost::lexical_cast<double>(tokens[2]);
                op.setTerminateTimeout(t);
            } else if (tokens[1] == "nocontacts") {
                op.setTerminateNoContacts();
            } else if (tokens[1] == "anycontacts") {
                op.setTerminateAnyContacts();
            } else {
                LOG_ERROR << "invalid token: " << tokens[1];
            }
        } else if (cmd == "dump") {
            LOG_INFO << endl << endl;
            LOG_INFO << kn()->saveXMLString();
            LOG_INFO << endl << endl;
        } else if (cmd == "select") {
            CHECK_EQ(n, 2) << "Invalid syntax: select <controller>";
            std::string name = tokens[1];
            LOG_INFO << "Select controller : " << name; 
            kn()->select(name.c_str());
            kn()->simpack()->reset();
            (*pResetMotions) = true;
        } else if (cmd == "changetarget") {
            std::string body = tokens[1];
            Eigen::VectorXd pose = parsePose(tokens[2].c_str());
            OpController op(kn());
            op.changeTargetPose(body.c_str(), pose);
            (*pResetMotions) = true;
        } else if (cmd == "export") {
            CHECK_EQ(n, 2) << "Invalid syntax: export <new name>";
            std::string newname = tokens[1];
            OpController op(kn());
            op.exportCon(newname.c_str());
            kn()->simpack()->reset();
            (*pResetMotions) = true;

        } else {
            if (n == 3) {
                std::string node0 = tokens[0];
                std::string dir   = tokens[1];
                std::string node1 = tokens[2];

                int n0 = parseNode(node0.c_str());
                Eigen::VectorXd d = parseDir(dir.c_str());
                int n1 = parseNode(node1.c_str());
                OpController op(kn());
                op.constraintPos(n0, n1, d);
            } else {
                return false;
            }
        }
        
        return true;
    }

    Eigen::VectorXd Interpreter::parseDir(const char* const dirname) {
        std::string name(dirname);
        const int DIM = 3;
        Eigen::VectorXd dir(Eigen::VectorXd::Zero(DIM));
        if (name == "infrontof" || name == "forward") {
            dir(0) = 1.0;
        } else {
            dir(0) = -1.0;
        }
        return dir;
    }

    int Interpreter:: parseNode(const char* const nodename) {
        std::string name(nodename);
        if (name == "root") return fullbody1::root;
        else if (name == "head") return fullbody1::head;
        else if (name == "lhand") return fullbody1::l_hand;
        else if (name == "rhand") return fullbody1::r_hand;
        else if (name == "lfoot") return fullbody1::l_toe;
        else if (name == "rfoot") return fullbody1::r_toe;

        return 0;
    }

    Eigen::VectorXd Interpreter::parsePose(const char* const posename) {
        std::string name(posename);
        int NDOFS = 33;
        Eigen::VectorXd ret = Eigen::VectorXd::Zero(NDOFS);
        if (name == "stand") {
            ret << 0, 0.97, 0, 0, 0, 0,
                0.2, 0.1, -0.5, 0.3, 0, 0,
                0.2, -0.1, -0.5, 0.3, 0, 0,
                0, -0.25, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0;
        } else if (name == "sit") {
            ret << -0.14, 0.56, 0.0, -0.13, 0.0, 0.0,
                1.4, 0.24, -2.0, 0.7, -0.03, -0.03,
                1.4, -0.24, -2.0, 0.7, 0.03, -0.03,
                0.0, -0.7, 0.0, 0.07, 0.0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0;
        } else {
            std::stringstream sin(posename);
            sin >> IO(ret);
            LOG_INFO << "Interpreter::parsePose result = " << IO(ret);
            // LOG_WARNING << "posename is invalid: " << posename;
        }
        return ret;
    }

// class Interpreter ends
////////////////////////////////////////////////////////////
    
    
} // namespace operation



