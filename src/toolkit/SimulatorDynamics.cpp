#include "Simulator.h"
#include "CppCommon.h"

// rtql8 headers
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/PointConstraint.h"
#include "collision/CollisionSkeleton.h"

namespace rtql8 {
    namespace toolkit {
        // Integration functions
        Eigen::VectorXd Simulator::getState() {
            Eigen::VectorXd state( nTotalDofs() * 2 );
            for (int i = 0, ptr = 0; i < nSkels(); i++) {
                int len = nDofs(i); // # Dofs of skeleton i
                state.segment(ptr, len) = getSkelState(i).q;
                state.segment(ptr + len, len) = getSkelState(i).qdot;
                ptr += (len * 2);
            }
            return state;
        }
        
        Eigen::VectorXd Simulator::evalDeriv() {

            Eigen::VectorXd deriv = Eigen::VectorXd::Zero( nTotalDofs() * 2 );
            for (int i = 0, ptr = 0; i < nSkels(); i++) {
                int len = nDofs(i); // # Dofs of skeleton i

                // Skip the immobile skeleton
                if (skel(i)->getImmobileState()) {
                    ptr += (len * 2);
                    continue;
                }
                // Shortcuts to q and qdot
                Eigen::VectorXd& q    = getSkelState(i).q;
                Eigen::VectorXd& qdot = getSkelState(i).qdot;

                // Calculate qddot
                Eigen::MatrixXd invM = skel(i)->getInvMassMatrix();
                Eigen::VectorXd C = skel(i)->getCombinedVector();
                Eigen::VectorXd cf = mCollision->getConstraintForce(i);
                Eigen::VectorXd tau = skel(i)->getInternalForces();
                Eigen::VectorXd ext = skel(i)->getExternalForces();
                // Eigen::VectorXd tau
                Eigen::VectorXd qddot = invM * (-C + cf + tau + ext);
                mConstraints[i]->applyConstraintForces(qddot);
                Eigen::VectorXd cstr = mConstraints[i]->getConstraintForce();
                qddot += invM * cstr;
                
                skel(i)->clampRotation(q, qdot);

                deriv.segment(ptr, len) = qdot;
                deriv.segment(ptr + len, len) = qddot;

                ptr += (len * 2);

                // if (getSimState().t < 0.0011) {
                //     cout << "Simulator::evalDeriv t = " << getSimState().t << endl;
                //     cout << "q = " << IO(q) << endl;
                //     cout << "qdot = " << IO(qdot) << endl;
                //     cout << "tau = " << IO(tau) << endl;
                //     cout << "cf = " << IO(cf) << endl;
                //     cout << endl;

                // }

                
            }
            return deriv;
        }
        
        void Simulator::setState(const Eigen::VectorXd& state){
            for (int i = 0, ptr = 0; i < nSkels(); i++) {
                int len = nDofs(i); // # Dofs of skeleton i
                mState.skels[i].q = state.segment(ptr, len);
                mState.skels[i].qdot = state.segment(ptr + len, len);

                mState.skels[i].expanded = 0;
                ptr += (len * 2);

            }
            mState.t += mConfig.Dt;

            // Update contacts
            {
                rtql8::collision_checking::SkeletonCollision* sc =
                    mCollision->getCollisionChecker();
                int nT = sc->getNumContact();
                rtql8::dynamics::ConstraintDynamics* dyn
                    = getConstraintDynamics(1);
                int nC = dyn->numConstraints();
                if (nC == 1) {
                    counterOneHand++;
                    if (counterOneHand < 100) {
                        nC = 0; // Wait until we have both contacts
                    }
                } else {
                    counterOneHand = 0;
                }

                Eigen::VectorXd contacts( (nT + nC) * 6);
                for (int i = 0; i < nT; i++) {
                    Eigen::Vector3d p = sc->getContact(i).point;
                    Eigen::Vector3d f = sc->getContact(i).force;
                    contacts.segment<3>(i * 6 + 0) = p;
                    contacts.segment<3>(i * 6 + 3) = f;
                }
                for (int i = 0; i < nC; i++) {
                    using rtql8::dynamics::PointConstraint;
                    PointConstraint* pc = (PointConstraint*)dyn->getConstraint(i);
                    Eigen::Vector3d p = pc->getTarget();
                    Eigen::Vector3d f = Eigen::Vector3d::Zero();
                    int j = nT + i;
                    contacts.segment<3>(j * 6 + 0) = p;
                    contacts.segment<3>(j * 6 + 3) = f;
                }
                
                mState.contacts = contacts;
            }
            // int nT = collision()->
            updateStructures();
        }

        void Simulator::expandSkelState(int skelId) {
            expandSkelState(skel(skelId), getSkelState(skelId));
        }

        void Simulator::expandSkelState(dynamics::SkeletonDynamics* skel, SkelState& sstate) {
            // Assume that the skeletons are matched with SkelState
            if (sstate.expanded == 1) {
                // This is already expanded skeleton state
                return;
            }

            // Expand COM
            sstate.COM = skel->getWorldCOM();
            sstate.MAXCOM = sstate.MAXCOM.cwiseMax(sstate.COM);
            sstate.MINCOM = sstate.MINCOM.cwiseMin(sstate.COM);

            const int DIM = 3;

            // Expand Linear Momentum
            {
                Eigen::MatrixXd J(Eigen::MatrixXd::Zero(DIM, skel->getNumDofs()));
                for (int i = 0; i < skel->getNumNodes(); i++) {
                    dynamics::BodyNodeDynamics *node = (dynamics::BodyNodeDynamics*)skel->getNode(i);
                    Eigen::MatrixXd localJ = node->getJacobianLinear() * node->getMass();
                    for (int j = 0; j < node->getNumDependentDofs(); j++) {
                        int index = node->getDependentDof(j);
                        J.col(index) += localJ.col(j);
                    }
                }
                sstate.P = J * sstate.qdot;
            }
            // Expand Angular Momentum
            {
                const Eigen::Vector3d& c = sstate.COM;
                Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                Eigen::Vector3d temp = Eigen::Vector3d::Zero();
                const Eigen::VectorXd& qdot = sstate.qdot;
                for (int i = 0; i < skel->getNumNodes(); i++) {
                    dynamics::BodyNodeDynamics *node
                        = (dynamics::BodyNodeDynamics*)skel->getNode(i);
                    node->evalVelocity(qdot);
                    node->evalOmega(qdot);
                    sum += node->getInertia() * node->mOmega;
                    sum += node->getMass() * (node->getWorldCOM() - c).cross(node->mVel);
                }
                sstate.L = sum;

            }

            // Expand COP
            {
                rtql8::collision_checking::SkeletonCollision* sc =
                    mCollision->getCollisionChecker();
                int nT = sc->getNumContact();
                Eigen::Vector3d COP(Eigen::Vector3d::Zero());

                for (int i = 0; i < nT; i++) {
                    Eigen::Vector3d p = sc->getContact(i).point;
                    COP += p; 
                }
                if (nT > 0) {
                    COP /= (double)nT;
                }
                
                sstate.COP = COP;
            }


            sstate.expanded = 1;
        }


    } // namespace toolkit
} // namespace rtql8
