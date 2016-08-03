/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_APP_CONTROLLER_H
#define controller_APP_CONTROLLER_H

#include <Eigen/Dense>
#include "toolkit/Toolkit.h"
#include "common/app_hppcommon.h"

namespace rtql8 {
    namespace kinematics {
        class BodyNode;
    } // namespace kinematics
} // namespace rtql8

namespace rapidxml {
    template<class Ch> class xml_document;
    template<class Ch> class xml_node;
} // namespace rapidxml

namespace controller {
    struct AppControllerState;
} // namespace controller

namespace controller {
    
    class AppController : public rtql8::toolkit::Controller {
    public:
        AppController();
        virtual ~AppController();

// Name
        std::string name;
        
// Inherited functions
        virtual int dim() { return 0; }
        virtual void setParams1D(double p) { setParams( (Eigen::VectorXd(1) << p).finished() ); }
        virtual void setParams(const Eigen::VectorXd& p) {}
        virtual Eigen::VectorXd computeTorque() { return zero(); }

// For the task-based parameters
        virtual void setParamsWithTask(double task) { setParams1D(task); }

// Controller state
        AppControllerState* conState() const { return conState_; }
        virtual void setConState(AppControllerState* _state) { conState_ = _state; }
        AppControllerState* conState_;

// Point the simulation and skeleton through Controller State Structure
        virtual rtql8::toolkit::Simulator* getSimulator() const;
        virtual int getSkelId() const;

// Auxiliary functions
        virtual std::string toString() const { return ""; }

// Activation
        virtual void reset();
        virtual bool activate();
        MEMBER_VAR(bool, activated);
        virtual double local_t();
        MEMBER_VAR(double, activated_time);

// PD Controls
    public:
        virtual void changePose() { /* Do Nothing */ }
        Eigen::VectorXd target();
        void setTarget(const Eigen::VectorXd& target);
        void adjustTarget(const Eigen::VectorXd& target);

        Eigen::VectorXd stiffness();
        void setStiffness(const Eigen::VectorXd& kp);

        Eigen::VectorXd computeSPDTorque();

// Virtual Forces
    public:
        virtual void exertForce() { /* Do Nothing */ }
        Eigen::VectorXd computeVFTorque();
    
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false) = 0; 

        
    }; // class AppController
    
} // namespace controller

#endif // #ifndef controller_APP_CONTROLLER_H

