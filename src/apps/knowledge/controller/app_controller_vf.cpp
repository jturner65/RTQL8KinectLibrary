/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "app_controller.h"
// rtql8 headers
#include "kinematics/BodyNode.h"
using rtql8::kinematics::BodyNode;

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

#include "common/app_cppcommon.h"
#include "app_controller_state.h"

namespace controller {
////////////////////////////////////////////////////////////
// class AppController : VF related functions

    Eigen::VectorXd apply(int nDofs,
                          BodyNode* node,
                          const Eigen::Vector3d& p0,
                          const Eigen::Vector3d& f) {
        using rtql8::utils::xformHom;
        
        Eigen::MatrixXd J(Eigen::MatrixXd::Zero(f.size(), nDofs));

        for (int i = 0; i < node->getNumDependentDofs(); i++) {
            int index = node->getDependentDof(i);
     
            Eigen::VectorXd jCol
                = xformHom(node->getDerivWorldTransform(i), p0);
            J.col(index) = jCol;
        }

        Eigen::VectorXd p = xformHom(node->getWorldTransform(), p0);


        Eigen::VectorXd vTorque = J.transpose() * f;
        return vTorque;
    }

    Eigen::VectorXd AppController::computeVFTorque() {
        const int NDOFS = nDofs();
        Eigen::VectorXd torque = zero();
        BOOST_FOREACH(const AppControllerState::VF& vf, conState()->vfs) {
            BodyNode* bn = node(vf.nodeId);
            Eigen::VectorXd vf_torque = apply(NDOFS, bn, vf.p, vf.f);
            torque += vf_torque;
        }
        return torque;
    }

// class AppController : VF related functions
////////////////////////////////////////////////////////////
} // namespace controller
