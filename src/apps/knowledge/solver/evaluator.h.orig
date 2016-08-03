/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_EVALUATOR_H
#define SOLVER_EVALUATOR_H

#include <vector>
#include <Eigen/Dense>
#include "common/app_hppcommon.h"

namespace controller {
    class Knowledge;
} // namespace controller

namespace solver {
    struct Sample;
    class Solver;
    class RemoteDB;
} // namespace solver

namespace solver {
    
    class Evaluator {
    public:
        Evaluator(Solver* _solver);
        virtual ~Evaluator();

        virtual bool init() { return true; }
        virtual bool evaluate(std::vector<Sample*>& samples, bool verbose = true) = 0;
        virtual bool reevaluate(std::vector<Sample*>& samples, bool verbose = true);
        virtual bool reevaluate(std::vector<Sample*>& samples,
                                const Eigen::VectorXd& tasks,
                                bool verbose = true);
            virtual bool destroy() { return true; }
    protected:
        MEMBER_PTR(Solver*, solver);
// Helper functions
        controller::Knowledge* kn();

        bool updateMyKnowledge();
        MEMBER_PTR(controller::Knowledge*, mykn);
        
    }; // class Evaluator


////////////////////////////////////////////////////////////
// class EvaluatorSingleCore
    class EvaluatorSingleCore : public Evaluator {
    public:
        EvaluatorSingleCore(Solver* _solver);
        virtual ~EvaluatorSingleCore();

        virtual bool init();
        virtual bool evaluate(std::vector<Sample*>& samples, bool verbose = true);
        virtual bool destroy();

    protected:
    }; // class EvaluatorSingleCore
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class EvaluatorRemoteDB
    class EvaluatorRemoteDB : public Evaluator {
    public:
        EvaluatorRemoteDB(Solver* _solver);
        virtual ~EvaluatorRemoteDB();

        virtual bool init();
        virtual bool evaluate(std::vector<Sample*>& samples, bool verbose = true);
        virtual bool destroy();
    protected:
        MEMBER_PTR(solver::RemoteDB*, remotedb);
            
    }; // class EvaluatorRemoteDB
//
////////////////////////////////////////////////////////////

    
} // namespace solver

#endif // #ifndef SOLVER_EVALUATOR_H

