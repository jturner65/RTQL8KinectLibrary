/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "app_controller_state.h"

#include <sstream>
#include "common/app_cppcommon.h"
#include "rapidxml/rapidxml.hpp"

namespace controller {
////////////////////////////////////////////////////////////
// class AppControllerState implementation
    AppControllerState::AppControllerState()
        : sim(NULL)
        , skelId(-2)
        , maxmove(DEFAULT_MAXMOVE)
        , hasTarget(false)
    {
    }
    
    AppControllerState::AppControllerState(int _nDofs)
        : MEMBER_INIT_ARG(nDofs)
        , sim(NULL)
        , skelId(-2)
        , maxmove(DEFAULT_MAXMOVE)
        , hasTarget(false)
    {
        target       = Eigen::VectorXd::Zero(nDofs());
        pushedTarget = Eigen::VectorXd::Zero(nDofs());
        qhat         = Eigen::VectorXd::Zero(nDofs());
        kp           = Eigen::VectorXd::Zero(nDofs());
        kd           = Eigen::VectorXd::Zero(nDofs());
    }

    AppControllerState::AppControllerState(const Eigen::VectorXd& _target,
                                           const Eigen::VectorXd& _kp,
                                           const Eigen::VectorXd& _kd)
        : target(_target)
        , pushedTarget(_target)
        , kp(_kp)
        , kd(_kd)
        , sim(NULL)
        , skelId(-2)
        , maxmove(DEFAULT_MAXMOVE)
        , hasTarget(false)
    {
        set_nDofs( target.size() );
    }

    AppControllerState::~AppControllerState() {
    }

// Auxiliary functions

    AppControllerState* AppControllerState::copy() const {
        AppControllerState* ret = new AppControllerState();
        (*ret) = (*this);
        return ret;
    }
    
    std::string AppControllerState::toString() const {
        std::stringstream sout;
        sout << "[CState: ";
        sout << IO(target) << endl;
        sout << IO(kp) << endl;
        sout << IO(kd) << endl;
        sout << "]";
        return sout.str();
    }

    AppControllerState::VF::VF(int _nodeId, double fx, double fy, double fz)
        : nodeId(_nodeId)
        , f(fx, fy, fz)
        , p(0.0, 0.0, 0.0)
    {
    }

// XML functions
    AppControllerState* AppControllerState::readXML(rapidxml::xml_node<char>* node) {
        return NULL;
    }
    
    rapidxml::xml_node<char>* AppControllerState::writeXML() const {
        return NULL;
    }

// class AppControllerState ends
////////////////////////////////////////////////////////////
    
    
} // namespace controller



