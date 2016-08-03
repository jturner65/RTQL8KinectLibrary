/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */


#include "app_composite_controller.h"
#include <boost/algorithm/string.hpp>
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "common/app_cppcommon.h"
#include "rapidxml_helper.h"

#include "knowledge.h"
#include "app_controller_state.h"
#include "predicate.h"
#include "cost.h"
#include "rig.h"
#include "common/fullbody1.h"

namespace controller {
////////////////////////////////////////////////////////////
// class AppCompositeController implementation
    AppCompositeController::AppCompositeController()
        : MEMBER_INIT_NULL(terminate)
        , MEMBER_INIT_NULL(cost)
        , previousActions(0)
        , currentPhaseIndex(0)
        , taskParam(0)
        , poseUpdated(false)
    {
        initpose = Eigen::VectorXd::Zero(0);
    }
    
    AppCompositeController::~AppCompositeController() {
        BOOST_FOREACH(AppController* c, childs) {
            delete c;
        }
        childs.clear();
        LOG_INFO << "delete " << name;
    }

    int AppCompositeController::dim() {
        return phases.size() + rigs.size();
    }
    
    void AppCompositeController::setParams(const Eigen::VectorXd& p) {
        if (p.size() != dim()) {
            LOG_WARNING << "!!!!";
            LOG_WARNING << "params.size() != dim() (" << p.size() << " vs. " << dim() << ")";
            LOG_WARNING << "dim() = " << phases.size() << " + " << rigs.size();

            BOOST_FOREACH(AppController* c, childs) {
                LOG_WARNING << "child = " << c->toString();
            }
        }

        int ptr = 0;
        for (int i = 0; i < phases.size(); i++) {
            AppCompositeController* ph = phases[i];
            ph->setParamsWithTask(p(ptr));
            ptr++;
        }
        for (int i = 0; i < rigs.size(); i++) {
            Rig* rig = rigs[i];
            rig->setParamsWithTask(p(ptr));
            ptr++;
         }
        
    }

    std::vector<std::string> AppCompositeController::getDimNames() {
        std::vector<std::string> names;
        BOOST_FOREACH(AppController* c, childs) {
            names.push_back(c->name);
        }
        return names;
    }

    Eigen::VectorXd AppCompositeController::upperBound() {
        int n = childs.size();
        int m = phases.size();
        Eigen::VectorXd bound = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < m; i++) {
            bound(i) = 1.0;
        }
        for (int i = m, j = 0; i < n; i++, j++) {
            Rig* r = rigs[j];
            bound(i) = r->upper();
        }
        return bound;
    }

    Eigen::VectorXd AppCompositeController::lowerBound() {
        int n = this->dim();
        int m = phases.size();
        Eigen::VectorXd bound = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < m; i++) {
            bound(i) = 0.0;
        }
        for (int i = m, j = 0; i < n; i++, j++) {
            Rig* r = rigs[j];
            bound(i) = r->lower();
        }
        return bound;
    }

// For the task-based parameters
    void AppCompositeController::setParamsWithTask(double task) {
        this->taskParam = task;
        for (int i = 0; i < childs.size(); i++) {
            AppController* con = childs[i];
            const Eigen::VectorXd& rg = regressions[i];
            double child_task = rg(0) + rg(1) * task;
            // LOG_INFO << boost::format("%.4lf = %.4lf + %.4lf * %.4lf")
            //     % child_task % rg(0) % rg(1) % task;
            con->setParamsWithTask(child_task);
        }

        // for (int i = 0; i < rigs.size(); i++) {
        //     Rig* rig = rigs[i];
        //     const Eigen::VectorXd& rg = rigs_regressions[i];
        //     double child_task = rg(0) + rg(1) * task;
        //     cout << boost::format("%.4lf = %.4lf + %.4lf * %.4lf")
        //         % child_task % rg(0) % rg(1) % task << endl;
        //     rig->setParamsWithTask(child_task);
        // }
        
    }

    void AppCompositeController::setRegressionParams(const Eigen::VectorXd& a,
                                                     const Eigen::VectorXd& b) {
        for (int i = 0; i < childs.size(); i++) {
            AppController* con = childs[i];
            Eigen::VectorXd rg = Eigen::VectorXd::Zero(2);
            rg(0) = a(i);
            rg(1) = b(i);
            regressions[i] = rg;
            LOG_INFO << boost::format("Child [%s] : %.4lf + t * %.4lf") % con->name % a(i) % b(i);
        }

    }
    

// For the final controller
    Eigen::VectorXd AppCompositeController::computeTorque() {
        proceedPhase();
        updateTargetPose();
        if (conState()->hasTarget == false) { // Cannot find any target pose
            // LOG_INFO << "cannot find the target pose " << name;
            // LOG_INFO << "target = initpose " << IO(this->initpose);
            conState()->hasTarget = true;
            conState()->pushedTarget = this->initpose;
            conState()->target       = conState()->pushedTarget;
        }
        // LOG_INFO << FUNCTION_NAME();

        std::vector<Predicate*> terminate_array;
        collectTerminates(terminate_array);
        // LOG_INFO << "t = " << t() << " " << local_t() << " " << activated_time();
        // LOG_INFO << "== terminates == ";
        BOOST_FOREACH(Predicate* t, terminate_array) {
            t->activate();
            // LOG_INFO << t->toString();
        }


        std::vector<Rig*> active_rigs;
        std::string path = collectActiveNames();
        collectRigs(active_rigs, path);
        // cout << "path = " << path << endl;

        // currentPhase()->activate();
        // LOG_INFO << "== active rigs ==";
        BOOST_FOREACH(Rig* r, active_rigs) {
            r->activate(); // When it is activated, r->changePose() will be called
            // LOG_INFO << r->toString();
        }

        conState()->resetAdjustedTarget();
        conState()->clearVirtualForces();
        BOOST_FOREACH(Rig* r, active_rigs) {
            r->exertForce();
        }
        if (path == "landing" || path == "rolljump.jump.landing") {
            // cout << "balancing.." << endl;
            using namespace fullbody1;
            const double MAX_F = 3000.0;
            AppControllerState::VF vf1(l_toe, 0, MAX_F * (-0.06), 0.0);
            AppControllerState::VF vf2(r_toe, 0, MAX_F * ( 0.04), 0.0);
            conState()->vfs.push_back(vf1);
            conState()->vfs.push_back(vf2);
            
        }

        Eigen::VectorXd torque = computeSPDTorque();
        // Eigen::VectorXd temp = (conState()->target).tail(5);
        // Eigen::VectorXd temp2 = torque.tail(5);
        // cout << "target = " << IO(temp  ) << endl;
        // cout << "torque = " << IO(temp2) << endl;
        torque += computeVFTorque();

        torque.head<6>().setZero();
        // cout << "cost = " << evaluate() << endl;
        // LOG_INFO << "torque = " << IO(torque);
        return torque;
    }

    void AppCompositeController::controlWithSPDOnly() {
        std::vector<Rig*> active_rigs;
        collectRigs(active_rigs);
        
        // currentPhase()->activate();
        BOOST_FOREACH(Rig* r, active_rigs) {
            r->activate(); // When it is activated, r->changePose() will be called
        }

        conState()->resetAdjustedTarget();
        conState()->clearVirtualForces();

        Eigen::VectorXd torque = computeSPDTorque();
        torque.head<6>().setZero();

        skel()->setInternalForces(torque);
    }
    
// Manipulation functions
    bool AppCompositeController::isTerminated() {
        // if (t() > TIME_LIMIT) {
        //     return true;
        // }

        // LOG_INFO << "[" << name << "] "
        //          << "index = " << currentPhaseIndex << " "
        //          << "phases.size() = " << phases.size() << " ";
        
        if (phases.size() > 0) {
            if (currentPhaseIndex >= phases.size()) {
                if (terminate()) {
                    return terminate()->value();
                } else {
                    return true;
                }
            }
            else {
                return false;
            }
        } else if (terminate()) {
            return terminate()->value();
        } else {
            return true; // should not be reached
        }
    }

    
    double AppCompositeController::evaluate() {
        if (t() > TIME_LIMIT_EVAL) {
            return COST_PENALTY;
        }
        
        if (cost()) {
            return cost()->value() + evaluateRigParams();
        }
        return evaluateRigParams();
    }

    double AppCompositeController::evaluate(double task) {
        if (t() > TIME_LIMIT_EVAL) {
            return COST_PENALTY;
        }
        if (cost()) {
            return cost()->value(task) + evaluateRigParams();
        }
        return evaluateRigParams();
    }

    void AppCompositeController::evaluateTaskAndError(double* task, double* error) {
        if (t() > TIME_LIMIT_EVAL) {
            *task = 0.5;
            *error = COST_PENALTY;
        }
        if (cost()) {
            cost()->evaluateTaskAndError(task, error);
            *error += evaluateRigParams();
        } else {
            *task  = -1.0;
            *error = -1.0;
        }
    }

    double AppCompositeController::evaluateRigParams() {
        double cost = 0.0;
        BOOST_FOREACH(Rig* r, rigs) {
            cost += r->cost();
        }
        return cost;
    }

    void AppCompositeController::reset() {
        AppController::reset();
        currentPhaseIndex = 0;
        poseUpdated = false;
        BOOST_FOREACH(AppController* c, childs) {
            c->reset();
        }
        if (conState()) {
            conState()->hasTarget = false;
            // LOG_INFO << "Reset the target pose as the initial pose!!!!!";
            // conState()->pushedTarget = this->initpose;
            // conState()->target       = conState()->pushedTarget;
            // LOG_INFO << "conState()->target = " << IO(conState()->target);
        }
        
    }

    bool AppCompositeController::shouldReleaseBar() {
        std::string path = collectActiveNames();
        if (boost::ends_with(path, "drop")) {
            return true;
        } else {
            return false;
        }
    }

// Pose functions
    Eigen::VectorXd AppCompositeController::getInitialTargetPose() {
        // if (phases.size() > 0) {
        if (previousActions > 0) {
            return phases[0]->getInitialTargetPose();
        } else {
            return inittargetpose;
        }
    }

    Eigen::VectorXd AppCompositeController::getCurrentTargetPose() {
        Eigen::VectorXd q = this->inittargetpose;
        if (q.size() > 1) {
            return q;
        }
        if (currentPhase()) {
            return currentPhase()->getCurrentTargetPose();
        } else {
            return this->initpose;
        }
    }

    void AppCompositeController::updateTargetPose() {
        if (poseUpdated) {
            return;
        }
        Eigen::VectorXd q = this->inittargetpose;
        if (isInPrevMotions() == false && q.size() > 1) {
            // LOG_INFO << name << " updated pose!!!!! ";
            // LOG_INFO << "target = " << IO(q);

            conState()->hasTarget = true;
            conState()->pushedTarget = this->inittargetpose;
            conState()->target       = conState()->pushedTarget;
            poseUpdated = true;
            return;
        } else if (currentPhase()) {
            currentPhase()->updateTargetPose();
        }

    }

    void AppCompositeController::setInitialPose(const Eigen::VectorXd& pose) {
        initpose = pose;
        inittargetpose = Eigen::VectorXd::Zero(1);
        // inittargetpose= pose;
    }

    void AppCompositeController::setInitialTargetPose(
        const Eigen::VectorXd& pose) {
        inittargetpose = pose;
        // LOG_INFO << "!!!!!!!!!!!!!!!!!!!";
        // LOG_INFO << IO(inittargetpose);
    }


// Controller state
    void AppCompositeController::setConState(AppControllerState* _state) {
        conState_ = _state;
        BOOST_FOREACH(AppCompositeController* ph, phases) {
            ph->setConState(_state);
        }
        BOOST_FOREACH(Rig* r, rigs) {
            r->setConState(_state);
        }
        if (terminate()) {
            terminate()->setConState(_state);
        }
        if (cost()) {
            cost()->setConState(_state);
        }
        // LOG_INFO << FUNCTION_NAME() << " OK";
    }


// Modification functions
    void AppCompositeController::addRig(Rig* r) {
        const int REG_PARAM_DIM = 2;
        r->setConState(conState());
        childs.push_back(r);
        regressions.push_back(Eigen::VectorXd::Zero(REG_PARAM_DIM));
        rigs.push_back(r);
        rigs_activateIndices.push_back("");
    }

    void AppCompositeController::addPrev(AppCompositeController* p) {
        const int REG_PARAM_DIM = 2;
        p->setConState(conState());
        childs.push_back(p);
        // regressions.push_back(Eigen::VectorXd::Zero(REG_PARAM_DIM));
        Eigen::VectorXd naive = Eigen::VectorXd::Zero(REG_PARAM_DIM);
        naive(0) = 0.0;
        naive(1) = 1.0;
        regressions.push_back(naive);
        phases.push_back(p);
        previousActions = phases.size(); // Mark this is a "PREVIOUS" action
    }

    AppCompositeController* AppCompositeController::flatten() {
        LOG_INFO << FUNCTION_NAME();

        // 1. Collect the params
        std::vector<AppCompositeController*> allphases;
        flatten(allphases);

        // 2. Calculate the regressions
        std::vector<Eigen::VectorXd> regressions;
        for (int i = 0; i < allphases.size(); i++) {
            AppCompositeController* p = allphases[i];
            const int REG_PARAM_DIM = 2;
            Eigen::VectorXd reg = Eigen::VectorXd::Zero(REG_PARAM_DIM);
            
            this->setParamsWithTask(0.0);
            reg(0) = p->getTaskParam();
            this->setParamsWithTask(1.0);
            reg(1) = p->getTaskParam() - reg(0);
            regressions.push_back(reg);

        }

        // 3. Remove prev actions
        for (int i = 0; i < allphases.size(); i++) {
            AppCompositeController* p = allphases[i];
            p->removePreviousActions();

            LOG_INFO << "Phase " << i << " : " << allphases[i]->name;
            LOG_INFO << "  " << IO(regressions[i]);
            LOG_INFO << "  " << allphases[i]->toString();

        }

        // 4. Create a new controller
        AppCompositeController* c = new AppCompositeController();
        c->set_cost( new Cost() );
        for (int i = 0; i < allphases.size(); i++) {
            AppCompositeController* p = allphases[i];
            Eigen::VectorXd rig = regressions[i];
            c->phases.push_back(p);
            c->childs.push_back(p);
            c->regressions.push_back(rig);
            // c->phase_poses.push_back(p->getInitialTargetPose());
            c->phase_poses.push_back(p->getInitialTargetPoseThis());
            if (i == 0) {
                c->inittargetpose = Eigen::VectorXd::Zero(1);
            } else if (i == allphases.size() - 1) {
                c->initpose       = p->initpose;
            }

        }        
        // LOG_INFO << "Result";
        // LOG_INFO << c->toString();

        return c;
    }

    void AppCompositeController::flatten(
        std::vector<AppCompositeController*>& allphases) {
        for (int i = 0; i < previousActions; i++) {
            phases[i]->flatten(allphases);
        }
        allphases.push_back(this);
    }

    void AppCompositeController::removePreviousActions() {
        for (int i = 0; i < previousActions; i++) {
            regressions.erase(regressions.begin());
            phases.erase(phases.begin());
            childs.erase(childs.begin());
            for (int j = 0; j < rigs_activateIndices.size(); j++) {
                std::string rindex = rigs_activateIndices[j];
                // if (rindex >= 0) rindex -= 1;
                rigs_activateIndices[j] = rindex;
            }
        }
        previousActions = 0;
    }

// Members and structures
    std::string AppCompositeController::collectActiveNames() {
        std::string path = name;
        if (currentPhase()) {
            path += ".";
            path += currentPhase()->collectActiveNames();
        }
        return path;
    }

    void AppCompositeController::collectRigs(std::vector<Rig*>& rig_array,
                                             std::string path) {
        if (currentPhase()) {
            currentPhase()->collectRigs(rig_array, path);
        }
        
        if (isIndexPrev(currentPhaseIndex) == false) {
            // BOOST_FOREACH(Rig* r, rigs) {
            // cout << "Path = [" << mypath << "]" << endl;
            int n = rigs.size();
            int m = rigs_activateIndices.size();
            if (n != m) {
                LOG_FATAL << "Unmatched rig informations :: " << n << " " << m;
                exit(0);
            }
            for (int i = 0; i < n; i++) {
                Rig* r = rigs[i];
                std::string idx = rigs_activateIndices[i];
                // if (idx.length() > 0) {
                //     cout << idx << " vs. " << path << endl;
                // }

                if (boost::algorithm::contains(path, idx) == false) {
                    continue;
                }
                // if (idx.length() > 0) {
                //     cout << idx << " at " << path << endl;
                // }
                rig_array.push_back(r);
            }
        } else { // If it is previous motion
            int n = rigs.size();
            int m = rigs_activateIndices.size();
            for (int i = 0; i < n; i++) {
                Rig* r = rigs[i];
                std::string idx = rigs_activateIndices[i];
                if (idx == "") {
                    continue;
                }
                // if (idx.length() > 0) {
                //     cout << idx << " vs. " << path << endl;
                // }

                // if (boost::algorithm::contains(path, idx) == false) {
                if (boost::algorithm::ends_with(path, idx) == false) {
                    continue;
                }
                // if (idx.length() > 0) {
                //     cout << idx << " at " << path << endl;
                // }
                rig_array.push_back(r);
            }
        }
    }

    void AppCompositeController::collectTerminates(std::vector<Predicate*>& terminate_array) {
        if (currentPhase()) {
            currentPhase()->collectTerminates(terminate_array);
        }
        // LOG_INFO << "[" << name << "] "
        //          << "phase = " << (currentPhase() ? currentPhase()->name : "--")
        //          << " (" << currentPhaseIndex << ") "
        //          << isIndexPrev(currentPhaseIndex);
        if (terminate() && isIndexPrev(currentPhaseIndex) == false) {
            terminate_array.push_back(terminate());
        }
    }

// Phase manipulation
    AppCompositeController* AppCompositeController::currentPhase() {
        // If this is a leaf node (= no phases), just return null
        if (phases.size() == 0) {
            return NULL;
        }

        if (currentPhaseIndex < 0) {
            return NULL; // Should not be reached, probably
        } else if (currentPhaseIndex < phases.size()) {
            return phases[currentPhaseIndex];
        } else {
            return NULL;
            // return phases[phases.size() - 1]; // Last index
        }
    }
    
    bool AppCompositeController::proceedPhase() {
        // LOG_INFO << "[" << name << "] proceedPhase: "
        //          << "currentPhase = "
        //          << ((currentPhase() != NULL) ? currentPhase()->name : "--")  << " ";
        if (currentPhase() == NULL) {
            return false;
        }
        if (currentPhaseIndex >= phases.size()) {
            return false;
        }
        bool childProceeded = currentPhase()->proceedPhase();
        if (childProceeded) {
            return false;
        }
        
        if (currentPhase()->isTerminated()) {
            // LOG_INFO << "time = " << t() << "/" << local_t();
            // LOG_INFO << "[" << name << "] "
            //          << "proceed phase: " << currentPhaseIndex << " -> "  << currentPhaseIndex + 1;

            currentPhaseIndex++;
            // if (currentPhaseIndex == phases.size()) {
            // // if (currentPhaseIndex == previousActions) {
            //     conState()->pushedTarget = this->inittargetpose;
            //     conState()->target       = conState()->pushedTarget;
            //     LOG_INFO << "Now enters the current controller!!! ["
            //              << name << "] at " << t();
            //     LOG_INFO << "target = " << IO(conState()->pushedTarget);
            // }
            return true;
        } else {
            return false;
        }
    }

// Auxiliary functions
    std::string AppCompositeController::toString() const {
        return toStringRecur(0);
    }
    std::string AppCompositeController::toStringRecur(int depth) const {
        std::string tab = "";
        for (int i = 0; i < depth; i++) tab += "    ";
        std::string str = tab;
        str += "[Composite " + name;

        if (terminate()) str += " " + terminate()->toString();
        if (cost())      str += " " + cost()->toString();

        int index = 0;
        BOOST_FOREACH(Rig* r, rigs) {
            str += " ";
            std::string rindex = rigs_activateIndices[index];
            if (rindex != "") {
                str += (boost::format("(%s)") % rindex).str();
            }
            str += r->toString();
            index++;
        }

        BOOST_FOREACH(AppCompositeController* ph, phases) {
            str += "\n" + ph->toStringRecur(depth + 1);
        }


        str +=  "]";
        return str;
    }

// XML functions
    AppCompositeController*
    AppCompositeController::readXML(Knowledge* kn,
                                    rapidxml::xml_node<char>* node,
                                    bool ignorePrev
        ) {
        namespace rx = rapidxml;
        name = rx::attr(node, "name");

        rapidxml::xml_node<char>* node_initpose = node->first_node("initialpose");
        if (node_initpose) {
            Eigen::VectorXd pose = rx::attr_vx(node_initpose, "target");
            setInitialPose(pose);
        }

        rapidxml::xml_node<char>* node_inittargetpose = node->first_node("initialtargetpose");
        if (node_inittargetpose) {
            Eigen::VectorXd pose = rx::attr_vx(node_inittargetpose, "target");
            setInitialTargetPose(pose);
            // LOG_INFO << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ";
            // LOG_INFO << IO(pose);
            // LOG_INFO << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ";
        }

        rapidxml::xml_node<char>* node_term = node->first_node("terminate");
        if (node_term) {
            set_terminate(
                Predicate::readXML(node_term->first_node())
                );
        }

        rapidxml::xml_node<char>* node_cost = node->first_node("evaluate");
        if (node_cost) {
            set_cost(
                (new Cost())->readXML(node_cost)
                );
        }

        int numPrevTags = 0;
        for (rx::xml_node<char> *child = node->first_node();
             child; child = child->next_sibling()) {
            std::string tag = rx::tag(child);
            // LOG_INFO << "child.tag = " << tag;
            if (tag != "rig" && tag != "phase" && tag != "prev") {
                continue;
            }
            if (ignorePrev && tag == "prev") {
                numPrevTags++;
                continue;
            }

            // cout << "***** " << name << " " << tag << " : " << ignorePrev << endl;
            
            rx::xml_node<char>* node_con = child->first_node("controller");
            if (node_con == NULL) {
                LOG_FATAL << "invalid xml structre: no controller node found";
                exit(1);
            }
            std::string con_name = rx::attr(node_con, "name");
            Eigen::VectorXd con_regression = rx::attr_vx(node_con, "regression");
            // LOG_INFO << "controller node: " << con_name << " "
            //          << "rig = " << IO(con_regression);

            AppController* con = NULL;
            if (tag == "phase") {
                con = kn->getControllerDupWithoutPrev(con_name.c_str());
            } else {
                con = kn->getControllerDup(con_name.c_str());
            }
            
            if (con == NULL) {
                LOG_FATAL << "cannot find the controller: " << con_name;
                exit(1);
            }
            
            childs.push_back(con);
            regressions.push_back(con_regression);
            
            if (tag == "rig") {
                Rig* r = dynamic_cast<Rig*>(con);
                if (rx::has_attr(node_con, "desired")) {
                    double p = rapidxml::attr_double(node_con, "desired");
                    r->setDesiredParam(p);
                    // LOG_INFO << "has desired: " << p;
                }
                std::string activateIndex = ""; // Activate for all cases
                if (rx::has_attr(child, "activateIndex")) {
                    activateIndex = rx::attr(child, "activateIndex");
                    // if (ignorePrev) activateIndex -= numPrevTags;

                    // LOG_INFO << "Activate index found!!! : " << activateIndex;
                }
                CHECK_NOTNULL(r) << " invalid type";
                rigs.push_back(r);
                rigs_activateIndices.push_back(activateIndex);

            } else if (tag == "phase" || tag == "prev") {
                AppCompositeController* c = dynamic_cast<AppCompositeController*>(con);
                CHECK_NOTNULL(c) << " invalid type";
                if (rx::has_attr(child, "target")) {
                    Eigen::VectorXd target = rx::attr_vx(child, "target");
                    c->setInitialTargetPose(target);
                    // LOG_INFO << "Target found!!! : " << IO(target);
                    phase_poses.push_back(target);
                } else {
                    phase_poses.push_back(Eigen::VectorXd::Zero(1));
                }

                phases.push_back(c);
                if (tag == "prev") {
                    previousActions = phases.size();
                    // LOG_INFO << "previous action = " << previousActions;
                }
            } 
        }

        if (has_attr(node, "set_task")) {
            double task = attr_double(node, "set_task");
            // LOG_INFO << "detect task " << task;
            setParamsWithTask(task);
            // LOG_INFO << "controller = " << endl << this->toString();
        }

        return this;
    }
    

    rapidxml::xml_node<char>* AppCompositeController::writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "composite");
        rx::add_attr(pdoc, node, "name",  name.c_str());
        if (!hashMode) {
            rx::add_attr(pdoc, node, "set_task",  taskParam);
        }

        {
            rx::xml_node<char>* node_initpose = rx::add_node(pdoc, node, "initialpose");
            rx::add_attr(pdoc, node_initpose, "target",  initpose);
        }
        if (inittargetpose.size() > 1) {
            rx::xml_node<char>* node_inittargetpose = rx::add_node(pdoc, node, "initialtargetpose");
            rx::add_attr(pdoc, node_inittargetpose, "target",  inittargetpose);
        }

        if (terminate()) {
            rx::xml_node<char>* node_terminate = rx::add_node(pdoc, node, "terminate");
            node_terminate->append_node( terminate()->writeXML(pdoc) );
        }
        if (cost()) {
            // rx::xml_node<char>* node_evaluate = rx::add_node(pdoc, node, "evaluate");
            node->append_node( cost()->writeXML(pdoc) );
        }

        int numPhases = 0;
        int numRigs = 0;
        for (int i = 0; i < childs.size(); i++) {
            rx::xml_node<char>* nd = NULL;
            if (isIndexPrev(i)) {
                nd = rx::add_node(pdoc, node, "prev");
                Eigen::VectorXd pp = phase_poses[numPhases];
                if (pp.size() > 1) {
                    rx::add_attr(pdoc, nd, "target", pp);
                }
                numPhases++;
            } else if (isIndexPhase(i)) {
                nd = rx::add_node(pdoc, node, "phase");
                Eigen::VectorXd pp = phase_poses[numPhases];
                if (pp.size() > 1) {
                    rx::add_attr(pdoc, nd, "target", pp);
                }
                numPhases++;
            } else {
                nd = rx::add_node(pdoc, node, "rig");
                std::string ai = rigs_activateIndices[numRigs];
                if (ai != "") {
                    rx::add_attr(pdoc, nd, "activateIndex", ai.c_str());
                }
                numRigs++;
            }
            rx::xml_node<char>* nd_con = rx::add_node(pdoc, nd, "controller");
            rx::add_attr(pdoc, nd_con, "name", childs[i]->name.c_str());
            rx::add_attr(pdoc, nd_con, "regression", regressions[i]);
            Rig* rig = dynamic_cast<Rig*>(childs[i]);
            if (rig != NULL && rig->hasDesiredParam()) {
                double p = rig->desiredParam();
                rx::add_attr(pdoc, nd_con, "desired", p);
            }
        }
        
        // for (int i = 0; i < phases.size(); i++) {
        //     rx::xml_node<char>* nd = rx::add_node(pdoc, node, "phase");
        //     rx::add_attr(pdoc, nd, "name", phases[i]->name.c_str());
        //     rx::add_attr(pdoc, nd, "regression", phases_regressions[i]);
        // }

        // rx::xml_node<char>* node_rig = rx::add_node(pdoc, node, "rig");
        // // BOOST_FOREACH(Rig* r, rigs) {
        // for (int i = 0; i < rigs.size(); i++) {
        //     Rig* r = rigs[i];
        //     Eigen::VectorXd reg = rigs_regressions[i];

        //     rx::xml_node<char>* nd = r->writeXML(pdoc);
        //     node_rig->append_node( nd );
        //     rx::add_attr(pdoc, nd, "regression", reg);
        // }

        
        return node;
    }

// class AppCompositeController ends
////////////////////////////////////////////////////////////
    
    
    
} // namespace controller



