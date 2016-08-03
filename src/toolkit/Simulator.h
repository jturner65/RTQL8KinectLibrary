#ifndef TOOLKIT_SIMULATOR_H
#define TOOLKIT_SIMULATOR_H

#include <vector>
#include "kinematics/FileInfoSkel.hpp"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "SimStates.h"

namespace rtql8 {
    namespace renderer {
        class RenderInterface;
    } // namespace renderer

    namespace dynamics {
        class SkeletonDynamics;
        class ContactDynamics;
        class ConstraintDynamics;
    } // namespace dynamics 

    namespace integration {
        class RK4Integrator;
    } // namespace integration

    namespace toolkit {
        class Simulator : public integration::IntegrableSystem {
        public:
            Simulator(const SimConfig& _config, bool verbose = true);
            virtual ~Simulator();
            // For the multi-process purpose
            Simulator* duplicate() const;
            void reset();

            // Simulation state functions
            void setSimConfig(const SimConfig& _config);
            SimConfig& getSimConfig();
            const SimConfig& getSimConfig() const;
            SkelConfig& getSkelConfig(int skelID);
            const SkelConfig& getSkelConfig(int skelID) const;

            SimState createEmptyState() const;
            void setSimState(const SimState& _state);
            SimState& getSimState();
            SkelState& getSkelState(int skelID);
            const SimState& getSimState() const;

            // Simulation execution functions
            void step();

            // Constraint functions
            dynamics::ConstraintDynamics* getConstraintDynamics(int i) {
                return mConstraints[i]; }
            void clearAllConstraints();


            // Accessor functions
            int nSkels() const;
            dynamics::SkeletonDynamics* skel(int skelID) const;
            int nTotalDofs() const;
            int nDofs(int skelID) const;
            double t() const;

            // Integration functions
            // implemented in SimulatorDynamics.cpp
            virtual Eigen::VectorXd getState();
            virtual Eigen::VectorXd evalDeriv();
            virtual void setState(const Eigen::VectorXd& state);	
        protected:
            void expandSkelState(int skelId);
            void expandSkelState(dynamics::SkeletonDynamics* skel, SkelState& sstate);

            // State variables;
            SimConfig mConfig;   // invariant state of simulation
            SimState mState;     // current state of simulation. Should be extended

            // Structure managing functions
            void initializeStructures(bool verbose = true);
            void destroyStructures();
            void updateStructures();
            
            // Required structures
            integration::Integrator* mIntegrator;
            dynamics::ContactDynamics* mCollision;
            std::vector<kinematics::FileInfoSkel<dynamics::SkeletonDynamics>*> mModels;
            std::vector<dynamics::ConstraintDynamics*> mConstraints;
            int counter;
            int counterOneHand;
        };

    } // namespace toolkit
} // namespace rtql8

#endif // #ifndef TOOLKIT_SIMULATOR_H
