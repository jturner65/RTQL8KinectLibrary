/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "sample.h"
#include "common/app_cppcommon.h"

#include <sstream>
#include "controller/app_composite_controller.h"
#include "controller/rig.h"
#include "controller/cost.h"
#include "controller/simpack.h"
#include "controller/knowledge.h"
#include "remotedb.h"

namespace solver {
////////////////////////////////////////////////////////////
// struct Sample implementation
    Sample::Sample(controller::AppCompositeController* con) {
        current    = con->name;
        dimensions = con->getDimNames();
        task       = 0.0;
        value      = 0.0;
        nextrig    = "";
        finalState = NULL;
        upper      =  Eigen::VectorXd::Ones( dimensions.size() );
        lower      = -Eigen::VectorXd::Ones( dimensions.size() );
        upper.head(con->dim()) = con->upperBound();
        lower.head(con->dim()) = con->lowerBound();
    }
    
    Sample::Sample(controller::AppCompositeController* con, std::string _next) {
        current    = con->name;
        dimensions = con->getDimNames();
        task       = 0.0;
        value      = 0.0;
        finalState = NULL;
        nextrig    = _next;
        dimensions.push_back(_next);
        upper      =  Eigen::VectorXd::Ones( dimensions.size() );
        lower      = -Eigen::VectorXd::Ones( dimensions.size() );
        upper.head(con->dim()) = con->upperBound();
        lower.head(con->dim()) = con->lowerBound();
    }

    Eigen::VectorXd Sample::generateRandomParams() {
        params = Eigen::VectorXd::Random(numDims());
        for (int j = 0; j < params.size(); j++) {
            double p = params(j);
            double w = (p + 1.0) / (2.0);
            double q = lower(j) + w * (upper(j) - lower(j));
            params(j) = q;
        }
        return params;
    }

    void Sample::evaluate(controller::Knowledge* kn) {
        using namespace controller;
        AppCompositeController* con = dynamic_cast<AppCompositeController*>(
            kn->getControllerDup(current.c_str())
            );

        if (nextrig.length() > 0) {
            controller::Rig* rig
                = dynamic_cast<controller::Rig*>(kn->getControllerDup(nextrig.c_str()));
            if (rig == NULL) {
                LOG_INFO << "cannot find the rig!!!!";
                exit(0);
            }
            con->addRig(rig);
        }
        
        con->setParams(this->params);

        SimPack* sp = kn->simpack();
        sp->set_con(con);
        sp->reset();
        // cout << "@@@@ " << IO(this->params) << endl;
        sp->simulate();
        // cout << "#### " << con->toString() << endl;

        this->final = sp->sim()->getSimState().toString();
        this->finalState = new rtql8::toolkit::SimState();
        bool loaded = this->finalState->fromString(this->final);
        // this->value = sp->evaluateController(con, this->finalState);
        sp->evaluateController(con, this->finalState, &(task), &(value));

        // delete con;
        // LOG_INFO << "evaluate OK";
    }

    void Sample::reevaluate(controller::Knowledge* kn) {
        using namespace controller;
        AppCompositeController* con = dynamic_cast<AppCompositeController*>(
            kn->getControllerDup(current.c_str())
            );

        if (nextrig.length() > 0) {
            controller::Rig* rig
                = dynamic_cast<controller::Rig*>(kn->getControllerDup(nextrig.c_str()));
            if (rig == NULL) {
                LOG_INFO << "cannot find the rig!!!!";
                exit(0);
            }
            con->addRig(rig);
        }
        
        SimPack* sp = kn->simpack();
        sp->set_con(con);
        sp->reset();
        con->setParams(params);

        this->finalState = new rtql8::toolkit::SimState();
        bool loaded = this->finalState->fromString(this->final);


        sp->evaluateController(con, this->finalState, &(task), &(value));
    }


    void Sample::reevaluate(controller::Knowledge* kn, const Eigen::VectorXd& tasks) {
        using namespace controller;
        AppCompositeController* con = dynamic_cast<AppCompositeController*>(
            kn->getControllerDup(current.c_str())
            );

        if (nextrig.length() > 0 && dimensions.size() > con->dim()) {
            controller::Rig* rig
                = dynamic_cast<controller::Rig*>(kn->getControllerDup(nextrig.c_str()));
            if (rig == NULL) {
                LOG_INFO << "cannot find the rig!!!!";
                exit(0);
            }
            con->addRig(rig);
        }

        SimPack* sp = kn->simpack();
        sp->set_con(con);
        sp->reset();
        con->setParams(params);
        // sp->simulate();

        this->finalState = new rtql8::toolkit::SimState();
        bool loaded = this->finalState->fromString(this->final);

        if (sp->con()->cost()->isRanged() == false) {
            double oldval = value;
            task = 0.0;
            // sp->evaluateController(con, this->finalState, task, &(value));
            this->tvalues = Eigen::VectorXd::Zero(tasks.size());
            for (int i = 0; i < tasks.size(); i++) {
                this->tvalues(i) = value;
            }
            cout << "value " << oldval << " --> " << value << endl;
        } else {
            this->tvalues = Eigen::VectorXd::Zero(tasks.size());

            double n        = tasks.size();
            double minTask  = -1.0;
            double minValue = 10e8;
            double avgValue = 0.0;
            for (int i = 0; i < tasks.size(); i++) {
                double task  = tasks(i);
                double value = 10e8;
                sp->evaluateController(con, this->finalState, task, &(value));
                this->tvalues(i) = value; // Record this value for future
                avgValue += (value / n);
                if (value < minValue) {
                    minValue = value;
                    minTask  = task;
                }
            }

            double ALPHA = 10.0;
            double BETA  =  0.1;
            double peakedness = (avgValue - minValue);
            double unpeakedness = std::max(BETA - peakedness, 0.0);

            // Penalize the task difference a little more
            bool USE_SHAPEANALYSIS = true;
            if (USE_SHAPEANALYSIS) {
                for (int i = 0; i < tasks.size(); i++) {
                    double task = tasks(i);
                    double v = this->tvalues(i);

                    v = v + ALPHA * unpeakedness * (task - minTask) * (task - minTask);
                    this->tvalues(i) = v;

                    // this->tvalues(i) += 2.0 * fabs(task - minTask);
                }
            }
            
            task  = minTask;
            value = minValue;
            
            // cout << "#### " << con->toString() << " : " << IO(tvalues) << endl;
        }

        
    }
    
    bool Sample::isEvaluated() {
        return (finalState != NULL);
    }

    void Sample::evaluateDB(controller::Knowledge* kn, RemoteDB* db) {
        // db->putSample(kn, this);
    }

    bool Sample::isEvaluatedDB(controller::Knowledge* kn, RemoteDB* db) {
        using namespace controller;
        if (finalState) {
            return true;
        }
        LOG_INFO << "fetching the result... ";
        bool result =  db->getSampleFinal(kn, this);
        LOG_INFO << "fetching the result... DONE";
        if (result) {
            updateFinalStateFromFinalString(kn);
        }
        return result;
    }

    void Sample::updateFinalStateFromFinalString(controller::Knowledge* kn) {
        using namespace controller;
        // LOG_INFO << "updating the final... ";

        // LOG_INFO << "##### " << "aaaaa kn = " << kn;
        // cout << kn->saveXMLString() << endl;
        if (this->finalState) delete this->finalState;
        this->finalState = new rtql8::toolkit::SimState();
        bool loaded = finalState->fromString(this->final);
        // LOG_INFO << "##### " << "bbbbb " << this->finalState;
        if (!loaded) {
            LOG_ERROR << "load final state from string failed";
            LOG_ERROR << "final string = " << final;
            LOG_ERROR << "final state = " << finalState->toString();
            LOG_ERROR << "params = " << IO(this->params);
            exit(0);

            return;
        }
        SimPack* sp = kn->simpack();
        // LOG_INFO << "##### " << "ccccc " << sp;

        sp->sim()->setSimState(*this->finalState);
        AppCompositeController* c = sp->con();
        // LOG_INFO << "##### " << "ddddd con = " << c;
        sp->evaluateController(c, this->finalState, &(task), &(value));
        // LOG_INFO << "updating the final... DONE";
        // LOG_INFO << "##### " << "eee";
        // value = kn->simpack()->evaluateController(c, this->finalState);
    }

    std::string Sample::toString() const {
        std::stringstream sout;
        sout << "[" << current << " (" << dimensions.size() << ") ";
        sout << "params = " << IO(params) << " : ";
        BOOST_FOREACH(const std::string& d, dimensions) {
            sout << " " << d;
        }
        sout << " final[" << final.length() << "]";
        // sout << " task = " << task;
        sout << boost::format(" task = %.3lf") % task;
        // sout << " value = " << value;
        sout << boost::format(" value = %.3lf") % value;
        sout << "]";
        return sout.str();
    }

// class Sample ends
////////////////////////////////////////////////////////////
    
} // namespace solver



