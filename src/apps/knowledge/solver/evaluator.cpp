/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "evaluator.h"
#include "common/app_cppcommon.h"

#include "controller/app_composite_controller.h"
#include "controller/knowledge.h"
#include "controller/simpack.h"
#include "boost/thread.hpp"

#include "sample.h"
#include "solver.h"
#include "remotedb.h"

namespace solver {
    
////////////////////////////////////////////////////////////
// class Evaluator implementation
    Evaluator::Evaluator(Solver* _solver)
        : MEMBER_INIT(solver, _solver)
        , MEMBER_INIT_NULL(mykn)
    {
    }
    
    Evaluator::~Evaluator() {
    }


    bool Evaluator::updateMyKnowledge() {
        LOG_INFO << FUNCTION_NAME() << "....";

        if (mykn()) {
            std::string lhs = mykn()->hashString();
            std::string rhs = kn()->hashString();
            LOG_INFO << "lhs = " << lhs;
            LOG_INFO << "rhs = " << rhs;
            if (lhs == rhs) {
                return false;
            }
        }
        set_mykn( kn()->duplicate() );
        // LOG_INFO << "mykn() = " << mykn()->saveXMLString();
        // mykn()->dumpControllerEntries();

        LOG_INFO << FUNCTION_NAME() << " OK";
        return true;
    }

    bool Evaluator::reevaluate(std::vector<Sample*>& samples, bool verbose) {
        updateMyKnowledge();

        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            // LOG_INFO << "re-evaluating sample [" << i << "]";
            s->reevaluate(mykn());
            LOG_INFO << "re-evaluating sample [" << i << "]... done";
            // Eigen::Vector3d COM = mykn()->simpack()->sim()->getSimState().skels[1].COM;
            // cout << "COM = " << IO(COM) << " task = " << s->task << " " << s->value << endl;
        }
        
        return true;
    }

    bool Evaluator::reevaluate(std::vector<Sample*>& samples,
                               const Eigen::VectorXd& tasks,
                               bool verbose) {
        updateMyKnowledge();

        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            // LOG_INFO << "re-evaluating sample [" << i << "]";
            s->reevaluate(mykn(), tasks);
            VLOG_INFO << "re-evaluating sample [" << i << "]... done";
            // Eigen::Vector3d COM = mykn()->simpack()->sim()->getSimState().skels[1].COM;
            // cout << "COM = " << IO(COM) << " task = " << s->task << " " << s->value << endl;
        }
        
        return true;
    }

    

// Helper functions
    controller::Knowledge* Evaluator::kn() {
        return solver()->kn();
    }


// class Evaluator ends
////////////////////////////////////////////////////////////
    
////////////////////////////////////////////////////////////
// class EvaluatorSingleCore implementation
    EvaluatorSingleCore::EvaluatorSingleCore(Solver* _solver)
        : Evaluator(_solver)
        // , MEMBER_INIT_NULL(mykn)
    {
        init();
    }
    
    EvaluatorSingleCore::~EvaluatorSingleCore() {
    }

    bool EvaluatorSingleCore::init() {
        updateMyKnowledge();

        return true;
    }

    bool EvaluatorSingleCore::evaluate(std::vector<Sample*>& samples, bool verbose) {
        updateMyKnowledge();

        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            LOG_INFO << "evaluating sample [" << i << "] : " << IO(s->params);
            s->evaluate(mykn());
            LOG_INFO << "evaluating sample [" << i << "]... done";
            // Eigen::Vector3d COM = mykn()->simpack()->sim()->getSimState().skels[1].COM;
            // cout << "COM = " << IO(COM) << " task = " << s->task << " " << s->value << endl;
        }
        
        return true;
    }

    bool EvaluatorSingleCore::destroy() {
        return true;
    }
    
// Mainloop

// class EvaluatorSingleCore ends
////////////////////////////////////////////////////////////
    
////////////////////////////////////////////////////////////
// class EvaluatorRemoteDB implementation
    EvaluatorRemoteDB::EvaluatorRemoteDB(Solver* _solver)
        : Evaluator(_solver)
        , MEMBER_INIT_NULL(remotedb)
    {
        init();
    }
    
    EvaluatorRemoteDB::~EvaluatorRemoteDB() {
    }

    bool EvaluatorRemoteDB::init() {
        updateMyKnowledge();


        // kn()->dumpControllerEntries();
        // mykn()->dumpControllerEntries();
        // cout << kn()->saveXMLString() << endl;
        // cout << mykn()->saveXMLString() << endl;
        // cout << "con = " << mykn()->simpack()->con() << endl;
        // exit(0);
        
        set_remotedb( new RemoteDB() );

        bool result = remotedb()->connect();
        remotedb()->registerKnowledge(mykn());
        remotedb()->deleteAllSamples();
        LOG_INFO << FUNCTION_NAME() << " OK";
        return true;
    }

    bool EvaluatorRemoteDB::evaluate(std::vector<Sample*>& samples, bool verbose) {
        updateMyKnowledge();
        remotedb()->registerKnowledge(mykn());

        remotedb()->prepareMultipleInsertionQuery();

        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            remotedb()->putSample(mykn(), s, i);
        }

        int lastID = remotedb()->executeMultipleInsertionQuery();
        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            s->dbid = lastID + i;
            // LOG_INFO << i << " : " << s->dbid;
        }
        // exit(0);
        
        // Wait for finishing evaluation
        for (int loop = 0; ;loop++) {
            std::vector<Sample*> todo_samples;
            for (int i = 0; i < samples.size(); i++) {
                Sample* s = samples[i];
                if (s->isEvaluated() == false) {
                    todo_samples.push_back(s);
                }
            }
            if (todo_samples.size() == 0) {
                LOG_INFO << "we have all samples evaluated!!! yay!";
                break;
            }

            std::vector<Sample*> evaluated_samples;
            remotedb()->queryMultipleSamplesFinal(todo_samples, &evaluated_samples);
            for (int i = 0; i < evaluated_samples.size(); i++) {
                Sample* s = evaluated_samples[i];
                LOG_INFO << "update [" << s->dbid << "]";
                s->updateFinalStateFromFinalString(mykn());
                loop = 0;
            }
            
            // for (int i = 0; i < samples.size(); i++) {

            //     Sample* s = samples[i];
            //     if (s->isEvaluatedDB( mykn(), remotedb() ) == false) {
            //         isFinished = false;
            //     }
            // }
            // if (isFinished) {
            //     LOG_INFO << "we have all samples evaluated!!! yay!";
            //     break;
            // }

            boost::posix_time::milliseconds t(1);  
            boost::this_thread::sleep(t);
            const int MAX_LOOP_COUNTER = 10000;
            if (loop > MAX_LOOP_COUNTER) {
                LOG_FATAL << "something wrong with evaluating.. reach the max loop";
                exit(0);
            }
        }

        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            // LOG_INFO << "evaluating sample [" << i << "]... done";
        }

        return true;
    }

    bool EvaluatorRemoteDB::destroy() {
        return true;
    }

// class EvaluatorRemoteDB ends
////////////////////////////////////////////////////////////
    

    
} // namespace solver



