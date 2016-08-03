/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_SAMPLE_H
#define SOLVER_SAMPLE_H

#include <string>
#include <vector>
#include <Eigen/Dense>

#include "common/app_hppcommon.h"

namespace rtql8 {
    namespace toolkit {
        struct SimState;
    } // namespace toolkit
} // namespace rtql8

namespace controller {
    class AppCompositeController;
    class Knowledge;
} // namespace controller

namespace solver {
    class RemoteDB;
} // namespace solver

namespace solver {
    
    struct Sample {
        std::string current;
        std::string nextrig;
        std::vector<std::string> dimensions;
        int numDims() { return dimensions.size(); }
        Eigen::VectorXd generateRandomParams();
        Eigen::VectorXd params;
        Eigen::VectorXd upper;
        Eigen::VectorXd lower;

        std::string final;
        rtql8::toolkit::SimState* finalState;
        double task;
        double value;
        Eigen::VectorXd tvalues; // Value for tasks

        int dbid;

        Sample(controller::AppCompositeController* con);
        Sample(controller::AppCompositeController* con, std::string _next);

        void evaluate(controller::Knowledge* kn);
        void reevaluate(controller::Knowledge* kn);
        void reevaluate(controller::Knowledge* kn, const Eigen::VectorXd& tasks);
        bool isEvaluated();

        void evaluateDB(controller::Knowledge* kn, RemoteDB* db);
        bool isEvaluatedDB(controller::Knowledge* kn, RemoteDB* db);
        void updateFinalStateFromFinalString(controller::Knowledge* kn);

        virtual std::string toString() const;
    }; // struct Sample
    
} // namespace solver

#endif // #ifndef SOLVER_SAMPLE_H

