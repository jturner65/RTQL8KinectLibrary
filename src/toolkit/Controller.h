#ifndef TOOLKIT_CONTROLLER_H
#define TOOLKIT_CONTROLLER_H

#include "SimStates.h"

namespace rtql8 {
    namespace dynamics {
        class SkeletonDynamics;
        class BodyNodeDynamics;
    } // namespace dynamics
    
    namespace toolkit {
        class Simulator;
        
        class Controller {
        public:
            Controller();
            Controller(Simulator* _sim, int _skelId);
            virtual ~Controller();
            virtual Controller* duplicate() const { return NULL; }
            
            virtual void setSimulator(Simulator* _sim);
            virtual Simulator* getSimulator() const;
            virtual void setSkelId(int skelId);
            virtual int getSkelId() const;
            virtual dynamics::SkeletonDynamics* skel() const;
            virtual const SimConfig& getSimConfig() const;
            virtual const SkelConfig& getSkelConfig() const;
            virtual const SimState& getSimState() const;
            virtual const SkelState& getSkelState() const;

        public:

            void control();
            virtual Eigen::VectorXd computeTorque() = 0;
        public:
            // Shortcuts!
            int             nDofs() const;             // Number of Dofs
            int             nNodes() const;            // Number of Nodes
            double          t() const;
            double          h() const;
            Eigen::VectorXd q() const;     // state in the generalized coord.
            Eigen::VectorXd qdot() const;  // velocity in the generalized coord.
            Eigen::MatrixXd M() const;     // Combined mass matrix
            Eigen::VectorXd C() const;     // Combined vector
            double          m() const;     // mass of the skeleton
            Eigen::Vector3d COM() const;   // Center of mass
            Eigen::Vector3d MAXCOM() const;    // Maximum of Center of mass
            Eigen::Vector3d MINCOM() const;    // Minimum of Center of mass
            Eigen::Vector3d COMdot() const; // Velocity of COM
            Eigen::Vector3d P() const;     // Linear momentum
            Eigen::Vector3d L() const;     // Angular momentum
            Eigen::Vector3d COP() const;   // Center of pressure
            int             nContacts() const; // Number of contacts
            rtql8::dynamics::BodyNodeDynamics* node(int nodeId) const;
            Eigen::Vector3d NODEPOS(int nodeId) const { return POS(nodeId); }
            Eigen::Vector3d POS(int nodeId) const;  // Position of the node
            Eigen::Vector3d APOS(int nNodes, ...) const;  // Average Position of the node
            Eigen::VectorXd zero() const;  // zero vector with nDofs length
            double          barDist() const;

            Eigen::Matrix3d getLocalFrame() const;
            Eigen::Vector3d LOCAL(const Eigen::Vector3d& v) const;

        protected:
            Simulator* mSim;
            int mSkelId;
            
        };

    } // namespace toolkit
} // namespace rtql8

#endif // #ifndef TOOLKIT_CONTROLLER_H
