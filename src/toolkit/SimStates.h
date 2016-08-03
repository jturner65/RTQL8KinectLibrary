#ifndef TOOLKIT_SKELSTATE_H
#define TOOLKIT_SKELSTATE_H

#include "State.hpp"

namespace rtql8 {
    namespace toolkit {
        struct SkelState;
        struct SimState;
        struct SimConfig;

        struct SkelState : public State {
            SkelState(int nDofs = 0) {
                registerMember("q", &q);
                registerMember("qdot", &qdot);
                registerMember("tau", &tau);

                q = Eigen::VectorXd::Zero(nDofs);
                qdot = Eigen::VectorXd::Zero(nDofs);
                tau = Eigen::VectorXd::Zero(nDofs);

                registerMember("expanded", &expanded);

                registerMember("COM", &COM);
                registerMember("MAXCOM", &MAXCOM);
                registerMember("MINCOM", &MINCOM);
                registerMember("P", &P);
                registerMember("L", &L);
                registerMember("COP", &COP);
                registerMember("nT", &nT);
                registerMember("minBarDist", &minBarDist);

                COM.setZero();
                const double MV = 9.9; // MAX_VALUE of COM
                MAXCOM = Eigen::Vector3d(-MV, -MV, -MV);
                MINCOM = Eigen::Vector3d( MV,  MV,  MV);
                P.setZero();
                L.setZero();
                COP.setZero();
                nT = 0;
                minBarDist = 10.0;
                expanded = 0;

            }
            
            // Required variables to proceed the simulation
            Eigen::VectorXd q;     // state
            Eigen::VectorXd qdot;  // velocity
            Eigen::VectorXd tau;   // torque (internal and external)

            // Generated variables from the simulation
            int expanded;         // Are the followings generated?
            
            Eigen::Vector3d COM;   // Center of mass
            Eigen::Vector3d MAXCOM;   // Center of mass
            Eigen::Vector3d MINCOM;   // Center of mass
            Eigen::Vector3d P;     // Linear momentum
            Eigen::Vector3d L;     // Angular momentum
            Eigen::Vector3d COP;   // Center of mass
            int nT;                // The number of contacts
            // Generated variables, but not registered since not human-interpretable
            // Still subject to assign/copy/duplicate/...
            Eigen::MatrixXd M;
            Eigen::VectorXd C;
            double minBarDist;
        }; // struct SkelState

        struct SimState : public State {
            SimState() {
                registerMember("t", &t);
                registerMember("skels", &skels);
                registerMember("contacts", &contacts);

                t = 0;
                contacts = Eigen::VectorXd::Zero(0);
            }

            // Member Variables
            double t;                    // Simulation time
            StateArray<SkelState> skels; // Skeleton states
            int nT() const { return contacts.size() / 6; } 
            Eigen::VectorXd contacts;

        }; // struct SimState


        struct SkelConfig : public State {
            SkelConfig() {
                registerMember("filename", &filename);
                registerMember("isImmobile", &isImmobile);
                registerMember("torqueLimit", &torqueLimit);
            }

            SkelConfig(const char* const _filename, bool _isImmobile = false) {
                registerMember("filename", &filename);
                registerMember("isImmobile", &isImmobile);
                registerMember("torqueLimit", &torqueLimit);

                filename = _filename;
                isImmobile = _isImmobile;
            }

            std::string filename;
            int isImmobile;
            Eigen::VectorXd torqueLimit;
        };
        
        struct SimConfig : public State {
            SimConfig() {
                registerMember("Dt", &Dt);
                registerMember("Mu", &Mu);
                registerMember("Cfm", &Cfm);
                registerMember("Gravity", &Gravity);
                registerMember("skels", &skels);
                registerMember("collisionRefreshRate", &collisionRefreshRate);
            }

            SimConfig(double _dt, double _mu, double _cfm)
                : Dt(_dt), Mu(_mu), Cfm(_cfm)
                , Gravity(Eigen::Vector3d(0.0, -9.81, 0.0))
                {
                registerMember("Dt", &Dt);
                registerMember("Mu", &Mu);
                registerMember("Cfm", &Cfm);
                registerMember("Gravity", &Gravity);
                registerMember("skels", &skels);
                registerMember("collisionRefreshRate", &collisionRefreshRate);
            }
            
            // Member Variables
            double Dt;       // Timestep
            double Mu;       // Friction coefficient
            double Cfm;      // Constraint force mixing
            int collisionRefreshRate;
            Eigen::Vector3d Gravity;  // Gravity
            StateArray<SkelConfig> skels; // Skeleton states
        };

        struct SimReplay : public State {
            SimReplay(const SimConfig& _config)
                : config(_config)
                {
                registerMember("config", &config);
                registerMember("frames", &frames);
            }
            SimConfig config;
            StateArray<SimState> frames;
        };

    } // namespace toolkit
} // namespace rtql8

#endif // #ifndef TOOLKIT_SKELSTATE_H
