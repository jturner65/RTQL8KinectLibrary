#include "Simulator.h"
#include "CppCommon.h"


// rtql8 headers
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/PointConstraint.h"

#define NUM_CONTACT_BASIS 4 // Usually, this is 4.

namespace rtql8 {
    namespace toolkit {
        Simulator::Simulator(const SimConfig& _config, bool verbose)
            : mConfig(_config)
            , mIntegrator(NULL)
            , mCollision(NULL)
        {
            mModels.clear();
            mConstraints.clear();
            initializeStructures(verbose);
        }
        
        Simulator::~Simulator() {
            destroyStructures();
        }

        // Simulation state functions
        void Simulator::setSimConfig(const SimConfig& _config) {
            mConfig = _config;
            destroyStructures();
            initializeStructures();
        }
        SimConfig& Simulator::getSimConfig() {
            return mConfig;
        }

        const SimConfig& Simulator::getSimConfig() const {
            return mConfig;
        }

        SkelConfig& Simulator::getSkelConfig(int skelID) {
            return mConfig.skels[skelID];
        }

        const SkelConfig& Simulator::getSkelConfig(int skelID) const {
            return mConfig.skels[skelID];
        }

        SimState Simulator::createEmptyState() const {
            SimState state;
            for (int i = 0; i < nSkels(); i++) {
                state.skels.push_back( SkelState( nDofs(i) ) );
            }
            return state;

        }

        void Simulator::setSimState(const SimState& _state) {
            mState = _state;
            updateStructures();
        }

        SimState& Simulator::getSimState() {
            return mState;
        }

        SkelState& Simulator::getSkelState(int skelID) {
            return mState.skels[skelID];
        }

        const SimState& Simulator::getSimState() const {
            return mState;
        }

        // Simulation execution functions
        void Simulator::step() {
            counter++;
            if (counter == mConfig.collisionRefreshRate) {
                // cout << "reboot.." << endl;
                if (mCollision) {
                    delete mCollision;
                }

                std::vector<dynamics::SkeletonDynamics*> skels; // For ContactDynamics
                for (int i = 0; i < nSkels(); i++) {
                    skels.push_back( skel(i) );
                }
            
                mCollision = new dynamics::ContactDynamics(
                    skels, mConfig.Dt, mConfig.Mu, NUM_CONTACT_BASIS);
                counter = 0;
            }
            mIntegrator->integrate(this, mConfig.Dt);
            
        }

        // Constraint functions
        void Simulator::clearAllConstraints() {
            for (int i = 0; i < mConstraints.size(); i++) {
                mConstraints[i]->clearConstraints();
            }
        }

        // Accessor functions
        int Simulator::nSkels() const {
            return mModels.size();
        }

        dynamics::SkeletonDynamics* Simulator::skel(int skelID) const {
            return mModels[skelID]->getSkel();
        }

        int Simulator::nTotalDofs() const {
            int n = 0;
            for (int i = 0; i < nSkels(); i++) {
                n += nDofs(i);
            }
            return n;
        }
        
        int Simulator::nDofs(int skelID) const {
            return skel(skelID)->getNumDofs();
        }

        double Simulator::t() const {
            return getSimState().t;
        }

        // For the multi-process purpose
        Simulator* Simulator::duplicate() const {
            Simulator* sim = new Simulator(this->getSimConfig());
            sim->setSimState(this->getSimState());
            return sim;
        }

        // Structure managing functions
        void Simulator::initializeStructures(bool verbose) {
            //  mIntegrator = new integration::RK4Integrator();
            mIntegrator = new integration::EulerIntegrator();

            mConstraints.clear();
            std::vector<dynamics::SkeletonDynamics*> skels; // For ContactDynamics

            for (int i = 0; i < mConfig.skels.size(); i++) {
                const SkelConfig& cfg = mConfig.skels[i];
                if (verbose) cout << "Skel " << i << ": filename = " << cfg.filename << endl;
                kinematics::FileInfoSkel<dynamics::SkeletonDynamics>* model
                    = new kinematics::FileInfoSkel<dynamics::SkeletonDynamics>();

                bool result = model->loadFile(cfg.filename.c_str(), kinematics::SKEL);
                dynamics::SkeletonDynamics* skel = model->getSkel();
                if (cfg.isImmobile == 1) {
                    skel->setImmobileState(true);
                }
                skel->initDynamics();

                if (!result) {
                    cerr << "NG: Failed to load skeleton " << cfg.filename << endl;
                    continue;
                } else {
                    if (verbose)
                        cout << "OK: load skeleton " << cfg.filename << " "
                             << "nDofs = " << skel->getNumDofs() << " "
                             << "Immobile = " << cfg.isImmobile << " "
                             << endl;
                }
                mModels.push_back(model);
                skels.push_back(skel);
                mConstraints.push_back(
                    new dynamics::ConstraintDynamics(skel)
                    );
            }
            
            counterOneHand = 0;
            mCollision = new dynamics::ContactDynamics(
                skels, mConfig.Dt, mConfig.Mu, NUM_CONTACT_BASIS);
            // mConstraint = new dynamics::ConstraintDynamics(
            //     skels.size(), mConfig.Dt);
            cout << "OK: # Skeleton = " << mModels.size() << endl;

        }

        void Simulator::reset() {
            counter = 0;
            counterOneHand = 0;
            if (mCollision) {
                delete mCollision;
            }

            std::vector<dynamics::SkeletonDynamics*> skels; // For ContactDynamics
            for (int i = 0; i < nSkels(); i++) {
                skels.push_back( skel(i) );
                skel(i)->clearInternalForces();
            }
            
            mCollision = new dynamics::ContactDynamics(
                skels, mConfig.Dt, mConfig.Mu, NUM_CONTACT_BASIS);

            for (int i = 0; i < mConstraints.size(); i++) {

                delete mConstraints[i];
                mConstraints[i] = new dynamics::ConstraintDynamics(skel(i));
            }

            // // Example code for adding a point constraint
            // using namespace dynamics;
            // BodyNodeDynamics* bd = dynamic_cast<BodyNodeDynamics*>(skels[1]->getNode(19));
            

            // PointConstraint *point1 = new PointConstraint(skels[1], bd, bd->getLocalCOM(), bd->getWorldCOM(), true, mConfig.Dt);

            // mConstraints[1]->addConstraint(point1);
            // /////////////////////////////////


            // if (mCollision == NULL) {
            //     mCollision = new dynamics::ContactDynamics(
            //         skels, mConfig.Dt, mConfig.Mu, NUM_CONTACT_BASIS);
            // }
        }
        
        void Simulator::destroyStructures() {
            // In the reverse order of the initialization
            SAFE_RELEASE_PTR(mCollision);
            for (int i = 0; i < mModels.size(); i++) {
                delete mModels[i];
            }
            mModels.clear();
            SAFE_RELEASE_PTR(mIntegrator);
            
        }

        void Simulator::updateStructures() {
            for (int i = 0; i < nSkels(); i++) {
                SkelState& state = getSkelState(i);

                if (skel(i)->getImmobileState()) {
                    skel(i)->setPose(state.q, true, false);
                } else {
                    skel(i)->setPose(state.q, false, true);
                    skel(i)->computeDynamics(mConfig.Gravity, state.qdot, true);

                    // if (getSimState().t < 0.0016) {
                    //     cout << "Simulator::updateStructures" << endl;
                    //     cout << "Skeleton " << i << endl;
                    //     Eigen::VectorXd C = skel(i)->getCombinedVector();
                    //     cout << IO(C) << endl;
                    // }
                    
                }

                expandSkelState(i);
            }

            mCollision->applyContactForces();

        }
        
    } // namespace toolkit
} // namespace rtql8
