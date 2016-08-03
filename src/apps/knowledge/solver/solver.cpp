/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "solver.h"
#include "common/app_cppcommon.h"
#include "explorer.h"
#include "evaluator.h"
#include "problem.h"

namespace solver {
    void workerFunc(void* ptr) {
        Solver* solver = static_cast<Solver*>(ptr);
        while(solver->stopRequested() == false) {
            cout << "working.." << endl;
            solver->mainloop();
            // cout << "resting.." << endl;
            // boost::posix_time::seconds t(1);  
            // boost::this_thread::sleep(t);
        }
    }
////////////////////////////////////////////////////////////
// class Solver implementation
    Solver::Solver(controller::Knowledge* _kn)
        : MEMBER_INIT(stopRequested, false)
        , MEMBER_INIT_ARG(kn)
        , MEMBER_INIT_NULL(exp)
        , MEMBER_INIT_NULL(prob)
        , MEMBER_INIT_NULL(evaluator)
        , MEMBER_INIT(evaluatorType, 0)
        , MEMBER_INIT(motionUpdatedIteration, -1)
        , MEMBER_INIT(clusterMode, 1)
        , MEMBER_INIT(clusterMinDist, 0.33)
    {
        // set_exp( new Explorer(_kn) );

        set_exp( new Explorer(this) );
    }
    
    Solver::~Solver() {
    }

// Thread manipulation
    bool Solver::run() {
        set_stopRequested(false);
        set_worker( new boost::thread(workerFunc, this) );
        LOG_INFO << FUNCTION_NAME();
        return true;
    }
    
    bool Solver::stop() {
        set_stopRequested(true);
        LOG_INFO << "solver stop requested. waiting for join...";
        worker()->join();
        MEMBER_RELEASE_PTR(worker);
        
        LOG_INFO << FUNCTION_NAME();
        return true;
    }

// Mainloop
    bool Solver::mainloop() {
        if (prob() && prob()->isSolved() == false) {
            cout << "prob()->solve" << endl;
            bool result =  prob()->solve(this);
            delete prob();
            set_prob(NULL);
            return result;
            // prob()->solve(this);
            // return true;
        } else {
            cout << "exp()->solve" << endl;
            return exp()->mainloop();
        }
    }

// Evaluator
    void Solver::setEvaluator() {
        if (evaluatorType() == 0) {
            setEvaluatorSingleCore();
        } else if (evaluatorType() == 2) {
            setEvaluatorRemoteDB();
        }
    }

    void Solver::setEvaluatorSingleCore() {
        MEMBER_RELEASE_PTR(evaluator);
        set_evaluator( new EvaluatorSingleCore(this) );
        LOG_INFO << FUNCTION_NAME();
    }

    void Solver::setEvaluatorRemoteDB() {
        MEMBER_RELEASE_PTR(evaluator);
        set_evaluator( new EvaluatorRemoteDB(this) );
        LOG_INFO << FUNCTION_NAME();
    }

// Set the optimization problem
    bool Solver::setProblem(solver::Problem* p) {
        bool ret = false;
        if (prob()) {
            MEMBER_RELEASE_PTR(prob);
            ret = true;
        }
        set_prob( p );
        return ret;
    }

    std::vector<Sample*>& Solver::getSamples() {
        return samples;
    }

    void Solver::replaceSamples(std::vector<Sample*>& new_samples) {
        samples = new_samples;
    }

    bool Solver::addSample(Sample* s) {
        samples.push_back(s);
        return true;
    }

// Style functions
    bool Solver::nextStyle() {
        if (prob()) {
            // prob()->set_nextStyleRequested(true);
            prob()->changeToNextStyle(this);
            return true;
        }
        return false;
    }

    int Solver::currentStyle() {
        if (prob()) {
            return prob()->currentStyle();
        }
        return 0;
    }
    
    int Solver::numStyles() {
        if (prob()) {
            return prob()->numStyles();
        }
        return 0;
    }

    bool Solver::terminateOpt() {
        if (prob()) {
            prob()->terminateOpt(this);
            return true;
        }
        return false;
    }

// class Solver ends
////////////////////////////////////////////////////////////
    
    
    
} // namespace solver



