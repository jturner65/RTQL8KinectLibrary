/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_EXPLORER_H
#define SOLVER_EXPLORER_H

#include <vector>
#include <string>
#include "common/app_hppcommon.h"

namespace controller {
    class Knowledge;
} // namespace controller

namespace solver {
    class Solver;
} // namespace solver


namespace solver {
    class Explorer {
    public:
        Explorer(Solver* _solver);
        virtual ~Explorer();

// Mainloop
        void filterRigNamesDupicated(std::vector<std::string>& names);
        void filterRigNamesByPrefix(std::vector<std::string>& names);
        bool mainloop();
        void dropAllSamples();
        void dropInvalidSamples();
    protected:
        MEMBER_PTR(Solver*, solver);
// Helper functions
        controller::Knowledge* kn();
        std::string rig_prefix;
    }; // class Explorer
    
} // namespace solver

#endif // #ifndef SOLVER_EXPLORER_H

