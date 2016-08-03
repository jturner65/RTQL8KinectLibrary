/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_APP_CONTROLLER_STATE_H
#define controller_APP_CONTROLLER_STATE_H

#include <vector>
#include <Eigen/Dense>
#include "common/app_hppcommon.h"

#define DEFAULT_MAXMOVE 0.16 // Will be loaded from xml

namespace rtql8 {
    namespace toolkit {
        class Simulator;
    } // namespace toolkit
} // namespace rtql8



namespace rapidxml {
    template<class Ch> class xml_node;
} // namespace rapidxml



namespace controller {
    
    struct AppControllerState {
        AppControllerState();
        AppControllerState(int _nDofs);
        AppControllerState(const Eigen::VectorXd& _target,
                           const Eigen::VectorXd& _kp,
                           const Eigen::VectorXd& _kd); 
        virtual ~AppControllerState();

// Point the simulation and skeleton
        rtql8::toolkit::Simulator* sim;
        int skelId;

// Auxiliary functions
        AppControllerState* copy() const;
        virtual std::string toString() const;

// PD Controls
        MEMBER_VAR(int, nDofs);
        Eigen::VectorXd target;
        Eigen::VectorXd pushedTarget; // reset every frame
        Eigen::VectorXd qhat;
        Eigen::VectorXd kp;
        Eigen::VectorXd kd;
        bool hasTarget;
        double maxmove;
        void resetAdjustedTarget() { target = pushedTarget; }

// Virtual Forces
        struct VF {
            int nodeId;
            Eigen::Vector3d p;
            Eigen::Vector3d f;

            VF(int _nodeId, double fx, double fy, double fz);
        };
        std::vector<VF> vfs;
        void clearVirtualForces() { vfs.clear(); } // Must be called at the beginning of iter.

// XML functions
        AppControllerState* readXML(rapidxml::xml_node<char>* node);
        rapidxml::xml_node<char>* writeXML() const;
    }; // class AppControllerState

    
    
} // namespace controller

#endif // #ifndef controller_APP_CONTROLLER_STATE_H

