/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "op_controller.h"
#include "common/app_cppcommon.h"
#include "common/fullbody1.h"
#include "controller/simpack.h"
#include "controller/knowledge.h"
#include "controller/rig.h"
#include "controller/app_composite_controller.h"
#include "controller/predicate.h"
#include "controller/cost.h"

namespace operation {
////////////////////////////////////////////////////////////
// class OpController implementation
    OpController::OpController(controller::Knowledge* _kn)
        : MEMBER_INIT_ARG(kn)
    {
    }
    
    OpController::~OpController() {
    }

    bool OpController::setTerminateTimeout(double t) {
        using namespace controller;
        LOG_INFO << "old = " << con()->toString();
        Predicate* old_pr = con()->terminate();
        if (old_pr) {
            delete old_pr;
        }

        PredicateTimeout* new_pr = new PredicateTimeout();
        new_pr->set_to(t);
        new_pr->setConState(con()->conState());
        con()->set_terminate(new_pr);

        LOG_INFO << "new = " << con()->toString();

        return true;
    }

    bool OpController::setTerminateNoContacts() {
        using namespace controller;
        LOG_INFO << "old = " << con()->toString();
        Predicate* old_pr = con()->terminate();
        if (old_pr) {
            delete old_pr;
        }

        PredicateTimeout* new_pr = new PredicateTimeout();
        new_pr->set_to(0.2);
        new_pr->setConState(con()->conState());
        con()->set_terminate(new_pr);

        LOG_INFO << "new = " << con()->toString();

        return true;
    }

    bool OpController::setTerminateAnyContacts() {
        using namespace controller;
        LOG_INFO << "old = " << con()->toString();
        Predicate* old_pr = con()->terminate();
        if (old_pr) {
            delete old_pr;
        }

        PredicateAnyContacts* new_pr = new PredicateAnyContacts();
        new_pr->setConState(con()->conState());
        con()->set_terminate(new_pr);

        LOG_INFO << "new = " << con()->toString();

        return true;
    }

    bool OpController::addRig(const char* const rig_name) {
        controller::Rig* rig
            = dynamic_cast<controller::Rig*>(kn()->getController(rig_name));
        CHECK_NOTNULL(rig) << "invalid rigname = " << rig_name;
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "rig = " << rig->toString();
        LOG_INFO << "prev con = " << con()->toString();

        con()->addRig(rig);

        LOG_INFO << "curr con = " << con()->toString();


        return true;
    }

    bool OpController::addRig(const char* const rig_name, double desiredParam) {
        controller::Rig* rig
            = dynamic_cast<controller::Rig*>(kn()->getController(rig_name));
        CHECK_NOTNULL(rig) << "invalid rigname = " << rig_name;
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "rig = " << rig->toString() << " with desiredParam " << desiredParam;
        LOG_INFO << "prev con = " << con()->toString();

        con()->addRig(rig);
        rig->setDesiredParam(desiredParam);

        LOG_INFO << "curr con = " << con()->toString();
        return true;
    }

    bool OpController::addPrev(const char* const prev_name) {
        controller::AppCompositeController* prev
            = dynamic_cast<controller::AppCompositeController*>(
                kn()->getController(prev_name));
        CHECK_NOTNULL(prev) << "invalid prevname = " << prev_name;
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "prev = " << prev->toString();
        LOG_INFO << "prev con = " << con()->toString();

        con()->addPrev(prev);

        if (con()->hasValidInitialPose() == false) {
            Eigen::VectorXd pose = prev->getInitialPose();
            con()->setInitialPose(pose);
        }
        // always
        {
            Eigen::VectorXd pose = prev->getInitialTargetPoseThis();
            LOG_INFO << "inittargetpose = " << IO(pose);
            con()->setInitialTargetPose(pose);
        }

        LOG_INFO << "curr con = " << con()->toString();
        kn()->simpack()->reset();

        return true;
    }

    bool OpController::addCostLanding() {
        LOG_INFO << FUNCTION_NAME();
        Eigen::Vector3d v0(Eigen::Vector3d::Zero());
        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_LANDING, v0);
        con()->cost()->addTerm(t);
        return true;
    }

    bool OpController::addCostUpright(double angleInDegree) {
        double a = angleInDegree / 180.0 * 3.141592; // Angle in radian
        double c = cos(a);
        double s = sin(a);
        Eigen::Vector3d v0(c, s, 0);
        LOG_INFO << FUNCTION_NAME() << "Dir = " << IO(v0);

        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_UPRIGHT, v0);
        con()->cost()->addTerm(t);
        return true;
    }

    bool OpController::addCostUpright2(double a0, double a1) {
        Eigen::Vector3d v0(a0, 0, 0);
        Eigen::Vector3d v1(a1, 0, 0);
        // {
        //     double a = a0 / 180.0 * 3.141592; // Angle in radian
        //     double c = cos(a);
        //     double s = sin(a);
        //     v0 = Eigen::Vector3d(c, s, 0);
        // }
        // {
        //     double a = a1 / 180.0 * 3.141592; // Angle in radian
        //     double c = cos(a);
        //     double s = sin(a);
        //     v1 = Eigen::Vector3d(c, s, 0);
        // }
        LOG_INFO << FUNCTION_NAME() << "Dir = " << IO(v0) << " to " << IO(v1);

        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_UPRIGHT, v0, v1);
        con()->cost()->addTerm(t);
        return true;
    }



    bool OpController::changeInitialPose(const Eigen::VectorXd& pose) {
        using namespace controller;
        using namespace fullbody1;
        LOG_INFO << FUNCTION_NAME();
        Eigen::VectorXd target = pose;
        for (int i = l_scapula_ex, j = r_scapula_ex;
             i < l_wrist_ex + 1; i++, j++) {
            if (i == l_elbow_ez) {
                target(i) = pose(j);
            } else {
                target(i) = -pose(j);
            }
        }
        LOG_INFO << "pose = " << IO(target);
        con()->setInitialPose(target);
        kn()->simpack()->reset();
        kn()->pushHint("clear");
        return true;
    }

    bool OpController::changeTargetPose(const char* const body,
                                        const Eigen::VectorXd& pose) {
        LOG_INFO << FUNCTION_NAME();
        std::string strbody(body);
        using namespace controller;
        using namespace fullbody1;
        Eigen::VectorXd target = con()->getInitialTargetPose();
        if (target.size() <= 1) {
            target = con()->getInitialPose();
        }
        
        if (strbody == "arms") {
            // for (int i = l_scapula_ex, j = r_scapula_ex;
            //      i < l_wrist_ex + 1; i++, j++) {
            //     if (i == l_elbow_ez) {
            //         target(i) = pose(j);
            //     } else {
            //         target(i) = -pose(j);
            //     }
            // }
            for (int i = l_scapula_ex; i < l_wrist_ex + 1; i++) {
                target(i) = pose(i);
            }
            for (int i = r_scapula_ex; i < r_wrist_ex + 1; i++) {
                target(i) = pose(i);
            }

        } else if (strbody == "legs") {
            int start = l_thigh_ez;
            int end   = r_toe_ez + 1;
            for (int i = start; i < end; i++) {
                target(i) = pose(i);
            }
        }
        LOG_INFO << body << ".. ";
        LOG_INFO << "pose = " << IO(target);
        con()->setInitialTargetPose(target);
        return true;
    }

    bool OpController::move(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1) {
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "v0 = " << IO(v0) << " v1 = " << IO(v1);
        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_COM, v0, v1);
        con()->cost()->addTerm(t);
        return true;
    }

    bool OpController::moveMax(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1) {
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "v0 = " << IO(v0) << " v1 = " << IO(v1);
        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_MAXCOM, v0, v1);
        con()->cost()->addTerm(t);
        return true;
    }

    bool OpController::moveMin(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1) {
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "v0 = " << IO(v0) << " v1 = " << IO(v1);
        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_MINCOM, v0, v1);
        con()->cost()->addTerm(t);
        return true;
    }

    bool OpController::speed(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1) {
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "v0 = " << IO(v0) << " v1 = " << IO(v1);
        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_COMDOT, v0, v1);
        con()->cost()->addTerm(t);
        return true;
    }

    bool OpController::spin(const Eigen::Vector3d& v0) {
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "v0 = " << IO(v0);
        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createCost(Cost::COST_TERM_L, v0);
        con()->cost()->addTerm(t);
        return true;
    }

    bool OpController::constraintPos(int node0, int node1, const Eigen::VectorXd& dir) {
        LOG_INFO << FUNCTION_NAME();
        LOG_INFO << "node0, 1 = " << node0 << ", " << node1 << " dir = " << IO(dir);

        using namespace controller;
        Cost::CostTerm* t = Cost::CostTerm::createConstraintPosition(dir, node0, node1);
        con()->cost()->addTerm(t);
        
        return true;
    }

    bool OpController::findCostTerm(int id,
                                    Eigen::Vector3d* v0,
                                    Eigen::Vector3d* v1) {

        LOG_INFO << FUNCTION_NAME() << " id = " << id;
        using namespace controller;
        return con()->cost()->findTerm((Cost::CostTermID)id, v0, v1);

    }

    bool OpController::findCOMTerm(Eigen::Vector3d* v0, Eigen::Vector3d* v1) {
        using namespace controller;
        return findCostTerm(Cost::COST_TERM_COM, v0, v1);
    }

    bool OpController::findCOMDotTerm(Eigen::Vector3d* v0, Eigen::Vector3d* v1) {
        using namespace controller;
        return findCostTerm(Cost::COST_TERM_COMDOT, v0, v1);
    }

    bool OpController::exportCon(const char* const newname) {
        using namespace controller;
        std::string name = con()->name;
        LOG_INFO << FUNCTION_NAME()
                 << " name  " << name
                 << " --> " << newname;
        AppCompositeController* c = dynamic_cast<AppCompositeController*>(
            kn()->getControllerDup(name.c_str())
            );
        AppCompositeController* new_c = c->flatten();
        new_c->name = newname;
        LOG_INFO << new_c->toString();
        // Add duplicated entry to the knowledge
        kn()->addControllerEntry(new_c, true);
        kn()->select(newname);

        return true;
    }

    controller::AppCompositeController* OpController::con() {
        return kn()->simpack()->con();
    }

// class OpController ends
////////////////////////////////////////////////////////////
    
    
    
} // namespace operation



