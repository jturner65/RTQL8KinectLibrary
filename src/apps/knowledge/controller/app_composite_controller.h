/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_APP_COMPOSITE_CONTROLLER_H
#define controller_APP_COMPOSITE_CONTROLLER_H

#include "app_controller.h"
#define TIME_LIMIT 3.0
#define TIME_LIMIT_EVAL 2.9

namespace rapidxml {
    template<class Ch> class xml_node;
    template<class Ch> class xml_document;
} // namespace rapidxml

namespace rtql8 {
    namespace toolkit {
        class Simulator;
    } // namespace toolkit
} // namespace rtql8

namespace controller {
    class Knowledge;
    struct AppControllerState;
    class Predicate;
    class Cost;
    class Rig;
} // namespace controller

namespace controller {
    
    class AppCompositeController : public AppController {
    public:
        AppCompositeController();
        virtual ~AppCompositeController();
    
        virtual int dim();
        virtual int numRigs() { return rigs.size(); }
        virtual int numPhases() { return phases.size(); }
        virtual void setParams(const Eigen::VectorXd& p);
        std::vector<std::string> getDimNames();
        Eigen::VectorXd upperBound();
        Eigen::VectorXd lowerBound();
// For the task-based parameters
        double getTaskParam() { return taskParam; }
        virtual void setParamsWithTask(double task);
        virtual void setRegressionParams(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

// Manipulation functions
        bool isTerminated();
        double evaluate();
        double evaluate(double task);
        void evaluateTaskAndError(double* task, double* error);
        double evaluateRigParams();
        void reset();

        bool shouldReleaseBar();

// Pose functions
        Eigen::VectorXd getInitialPose() { return initpose; }
        Eigen::VectorXd getInitialTargetPose();
        Eigen::VectorXd getCurrentTargetPose();
        Eigen::VectorXd getInitialTargetPoseThis() { return inittargetpose; }
        void setInitialPose(const Eigen::VectorXd& pose);
        void setInitialTargetPose(const Eigen::VectorXd& pose);
        bool hasValidInitialPose() { return (initpose.size() > 0); } 
        void updateTargetPose();
        bool poseUpdated;

// For the final controller
        Eigen::VectorXd finalTorque();
        void applyTorqueLimit(Eigen::VectorXd& torque);
        virtual Eigen::VectorXd computeTorque();
        void controlWithSPDOnly();
// Controller state
        virtual void setConState(AppControllerState* _state);

// Modification functions
        void addRig(Rig* r);
        void addPrev(AppCompositeController* p);

        AppCompositeController* flatten();
        void flatten(std::vector<AppCompositeController*>& allphases);
        void removePreviousActions();

    protected:
// Members and structures
        double taskParam;
        MEMBER_PTR(Predicate*, terminate);
        MEMBER_PTR(Cost*, cost);
        Eigen::VectorXd initpose;
        Eigen::VectorXd inittargetpose;
        std::vector<AppController*> childs;
        std::vector<AppCompositeController*> phases;
        std::vector<Eigen::VectorXd> phase_poses;
        std::vector<Rig*> rigs;
        std::vector<std::string> rigs_activateIndices;
        std::vector<Eigen::VectorXd> regressions;

        std::string collectActiveNames();
        void collectRigs(std::vector<Rig*>& rig_array, std::string path="");
        void collectTerminates(std::vector<Predicate*>& terminate_array);
    
// Phase manipulation
        int previousActions;
        int currentPhaseIndex;
        AppCompositeController* currentPhase();
        bool proceedPhase();
        bool isInPrevMotions() { return isIndexPrev(currentPhaseIndex); }
        bool isIndexPrev(int i)  { return (i < previousActions); }
        bool isIndexPhase(int i) { return (i < phases.size()); }
        
    public:
// Auxiliary functions
        virtual std::string toString() const;
        std::string toStringRecur(int depth) const;
// XML functions
        AppCompositeController* readXML(Knowledge* kn,
                                        rapidxml::xml_node<char>* node,
                                        bool ignorePrev = false);
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc,
                                                   bool hashMode = false);

    }; // class AppCompositeController
    
} // namespace controller

#endif // #ifndef controller_APP_COMPOSITE_CONTROLLER_H

