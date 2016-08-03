/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "app_controller.h"

// rtql8 headers
#include "kinematics/Dof.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"

#include "common/app_cppcommon.h"

#include "app_controller_state.h"

namespace controller {
////////////////////////////////////////////////////////////
// class AppController : SPD related functions
    
    Eigen::VectorXd regulateMaximumMove(const Eigen::VectorXd& pose,
                                        const Eigen::VectorXd& target,
                                        double maxdist) {
        // cout << "maxdist = " << maxdist << endl;
        const double MAX_DIST = maxdist;
        Eigen::VectorXd ret = target;
        for (int i = 0; i < pose.size(); i++) {
            if (ret(i) < pose(i) - MAX_DIST) {
                ret(i) = pose(i) - MAX_DIST;
            }
            if (ret(i) > pose(i) + MAX_DIST) {
                ret(i) = pose(i) + MAX_DIST;
            }
        }
        return ret;
    }


    Eigen::VectorXd regulateTarget(rtql8::dynamics::SkeletonDynamics* skel,
                                   const Eigen::VectorXd& target) {
        Eigen::VectorXd ret = target;
        for (int i = 0; i < skel->getNumDofs(); i++) {
            rtql8::kinematics::Dof* dof = skel->getDof(i);
            double lower = dof->getMin();
            double upper = dof->getMax();
            // if (target(i) < lower) {
            //     cout << "joint " << i << " : " << target(i) << " < "
            //          << lower << endl;
            // }
            // if (target(i) > upper) {
            //     cout << "joint " << i << " : " << target(i) << " > "
            //          << upper << endl;
            // }
            ret(i) = CONFINE(target(i), lower, upper);
        }
        return ret;
    }
    
    void regulatePD(rtql8::dynamics::SkeletonDynamics* skel,
                    const Eigen::VectorXd& q,
                    const Eigen::VectorXd& qdot,
                    const double h,
                    const Eigen::MatrixXd& KP,
                    const Eigen::MatrixXd& KD,
                    Eigen::VectorXd& pterm,
                    Eigen::VectorXd& dterm) {

        for (int i = 0; i < skel->getNumDofs(); i++) {
            const double EPS = 0.001;
            rtql8::kinematics::Dof* dof = skel->getDof(i);
            double lower = dof->getMin();
            double upper = dof->getMax();
            double value = q(i);
            // If the dof in the desired range 
            if (lower - EPS < value && value < upper + EPS) {
                continue;
            }
            double PSCALE = 10.0;
            double DSCALE = 3.0;
             // If the dof is less than the lower bound and still decreasing
            if (value <= lower - EPS && qdot(i) < 0.0) {
                pterm(i) = PSCALE * (-KP(i, i)) * (q(i) + qdot(i) * h - lower);
                dterm(i) = DSCALE * (-KD(i, i)) * qdot(i);
                // cout << dof->getName() << " : ";
                // cout << lower << " < " << value << " < " << upper << endl;
            }
            // If the dof is greater than the upper bound and still increasing
            if (upper + EPS <= value && 0.0 < qdot(i) ) {
                pterm(i) = PSCALE * (-KP(i, i)) * (q(i) + qdot(i) * h - upper);
                dterm(i) = DSCALE * (-KD(i, i)) * qdot(i);
                // cout << dof->getName() << " : ";
                // cout << lower << " < " << value << " < " << upper << endl;
            }
        }
        
    }

    int g_count = 0;
    Eigen::VectorXd AppController::computeSPDTorque() {
        g_count++;
        // if ((g_count % 50) == 0) {
        //     cout << "AA. targt = " << IO(conState()->target) << endl;
        // }
        Eigen::VectorXd qhat = regulateTarget( skel(), conState()->target );
        // if ((g_count % 50) == 0) {
        //     cout << "BB. regulated = " << IO(qhat) << endl;
        // }
        double maxmove = conState()->maxmove;
        qhat = regulateMaximumMove(q(), qhat, maxmove);
        // if ((g_count % 50) == 0) {
        //     cout << "CC. maxmove = " << IO(qhat) << endl;
        // }
        conState()->qhat = qhat; // For debugging
        Eigen::MatrixXd KP = conState()->kp.asDiagonal();
        Eigen::MatrixXd KD = conState()->kd.asDiagonal();

        Eigen::MatrixXd A = M() + KD * h();
        Eigen::VectorXd pterm = -KP * (q() + qdot() * h() - qhat);
        Eigen::VectorXd dterm = -KD * qdot();

        regulatePD( skel(), q(), qdot(), h(), KP, KD,
                    pterm, dterm);

        Eigen::VectorXd B = -C() + pterm + dterm;
        Eigen::VectorXd qddot = A.colPivHouseholderQr().solve(B);


        Eigen::VectorXd tau = pterm + dterm - KD * qddot * h();
        return tau;
    }


// class AppController : SPD related functions
////////////////////////////////////////////////////////////
} // namespace controller
