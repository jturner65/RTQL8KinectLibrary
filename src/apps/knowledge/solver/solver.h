/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_SOLVER_H
#define SOLVER_SOLVER_H

#include <boost/thread.hpp>
#include <Eigen/Dense>
#include "common/app_hppcommon.h"

namespace controller {
    class Knowledge;
} // namespace controller

namespace solver {
    struct Sample;
    class Explorer;
    class Problem;
    class Evaluator;
} // namespace solver

namespace solver {
    /*
      Solver class runs at the background using a single thread
     */
    class Solver {
    public:
        Solver(controller::Knowledge* _kn);
        virtual ~Solver();

// Thread manipulation
        bool run();
        bool stop();
// Mainloop
        bool mainloop();
// Evaluator
        void setEvaluator();
        void setEvaluatorSingleCore();
        void setEvaluatorRemoteDB();
// Set the optimization problem
        bool setProblem(solver::Problem* p);
// Miscs
        int numSamples() { return samples.size(); }
        std::vector<Sample*>& getSamples();
        void replaceSamples(std::vector<Sample*>& new_samples);
        bool addSample(Sample* s);
        bool isMotionUpdated() { return motionUpdatedIteration() < 0; }
// Style functions
        bool nextStyle();
        int currentStyle();
        int numStyles();
        bool terminateOpt();
    protected:
        MEMBER_PTR(boost::thread*, worker);
        MEMBER_VAR(bool, stopRequested);

        MEMBER_PTR(controller::Knowledge*, kn);
        MEMBER_PTR(solver::Explorer*, exp);
        MEMBER_PTR(solver::Problem*, prob);

        MEMBER_VAR(int, evaluatorType);
        MEMBER_PTR(solver::Evaluator*, evaluator);
        std::vector<Sample*> samples;

        // Did the 
        MEMBER_VAR(int, motionUpdatedIteration);

        // Options, which will be used by children
        MEMBER_VAR(int, mu);
        MEMBER_VAR(int, lambda);
        MEMBER_VAR(double, sigma);
        MEMBER_VAR(Eigen::VectorXd, sampled_tasks);
        MEMBER_VAR(int, clusterMode);
        MEMBER_VAR(double, clusterMinDist);
    }; // class Solver
    
} // namespace solver

#endif // #ifndef SOLVER_SOLVER_H

