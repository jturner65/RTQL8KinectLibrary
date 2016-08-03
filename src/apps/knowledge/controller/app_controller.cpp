/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "app_controller.h"
#include "common/app_cppcommon.h"

#include "app_controller_state.h"

namespace controller {
////////////////////////////////////////////////////////////
// class AppController implementation
    AppController::AppController()
        : conState_(NULL)
        , MEMBER_INIT(activated, false)
    {
    }
    
    AppController::~AppController() {
    }

// Point the simulation and skeleton through Controller State Structure
    rtql8::toolkit::Simulator* AppController::getSimulator() const {
        return conState()->sim;
    }

    int AppController::getSkelId() const {
        return conState()->skelId;
    }

// Activation
    void AppController::reset() {
        set_activated(false);
        set_activated_time(0.0);
        // LOG_INFO << "[" << name << "] reset(): activated = " << activated_time();
    }
    
    bool AppController::activate() {
        if (activated()) {
            return false;
        }
        set_activated(true);
        set_activated_time(t());
        // LOG_INFO << "[" << name << "] activated_time = " << activated_time();
        // If it is activated
        changePose();
        return true;
    }

    double AppController::local_t() {
        return t() - activated_time();
    }

// PD Controls
    Eigen::VectorXd AppController::target() {
        return conState()->target;
    }

    void AppController::setTarget(const Eigen::VectorXd& target) {
        conState()->target = target;
        conState()->pushedTarget = target;
    }

    void AppController::adjustTarget(const Eigen::VectorXd& target) {
        conState()->target = target;
    }

    Eigen::VectorXd AppController::stiffness() {
        return conState()->kp;
    }
    
    void AppController::setStiffness(const Eigen::VectorXd& kp) {
        conState()->kp = kp;
    }


// class AppController ends
////////////////////////////////////////////////////////////
    
    
    
} // namespace controller



