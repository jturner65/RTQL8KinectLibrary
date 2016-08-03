/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_PROBLEM_H
#define SOLVER_PROBLEM_H

#include <Eigen/Dense>
#include <vector>
#include <set>
#include "common/app_hppcommon.h"

namespace controller {
    class Knowledge;
} // namespace controller

namespace solver {
    struct Sample;
    struct ModelCombined;
    class Solver;
} // namespace solver

namespace solver {

    class Problem {
    public:
        Problem();
        virtual ~Problem();

        bool solve(Solver* solver);
        bool solve2(Solver* solver); // Two points version
        bool solve3(Solver* solver); // Regression version
        bool solve_simple(Solver* solver); // Test...
        void solve3_regression(ModelCombined* md,
                               Solver* solver,
                               std::vector<Sample*>& samples);
        void solve3_cov(ModelCombined* md,
                        int index,
                        Solver* solver,
                        std::vector<Sample*>& samples);
        ModelCombined* solve3_create_model(Solver* solver,
                                           std::vector<Sample*>& samples);
        std::vector<ModelCombined*> solve3_create_models_cluster(
            Solver* solver, std::vector<Sample*>& samples);
        void solve3_update_model_params(ModelCombined* md, Solver* solver);
        bool solve3_adapt_model(ModelCombined* md, Solver* solver);

        
        std::vector<Sample*> filterUnmatchedSamples(controller::Knowledge* kn,
                                                    const std::vector<Sample*>& samples);
        std::vector<Sample*> filterInvalidSamples(const std::vector<Sample*>& samples);
        std::vector<Sample*> filterBadSamples(const std::vector<Sample*>& samples);
        std::vector<Sample*> selectByClosestTask(const std::vector<Sample*>& samples,
                                                 const Eigen::VectorXd& tasks,
                                                 int targetIndex);
        std::vector<Sample*> selectByTaskValue(std::vector<Sample*>& samples,
                                               int idx, int min_num, int max_num);
        std::vector<Sample*> selectByTaskRange(std::vector<Sample*>& samples,
                                               double lo, double hi,
                                               int min_num, int max_num);
        std::vector<Sample*> add(std::vector<Sample*>& s0,
                                 std::vector<Sample*>& s1,
                                 std::vector<Sample*>& s2);

        bool isRanged(Solver* solver);
        bool isDegenerated(Solver* solver);

        void dumpSamples(const std::vector<Sample*>& samples);
        void dumpSamples(const std::set<Sample*>& samples);
        int randomByVolumns(const std::vector<double>& volumns);

        int currentStyle() { return currentModelIndex; }
        int numStyles() { return models.size(); }
        int changeToNextStyle(Solver* solver);
        int terminateOpt(Solver* solver);
    protected:
        int currentModelIndex;
        std::vector<ModelCombined*> models;
        
        MEMBER_VAR(bool, isSolved);
        MEMBER_VAR(bool, nextStyleRequested);
        MEMBER_VAR(bool, terminateRequested);
    }; // class Problem
    
} // namespace solver

#endif // #ifndef SOLVER_PROBLEM_H

