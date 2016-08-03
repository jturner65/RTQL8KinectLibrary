/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef OPERATION_INTERPRETER_H
#define OPERATION_INTERPRETER_H

#include <Eigen/Dense>
#include "common/app_hppcommon.h"

namespace controller {
    class Knowledge;
} // namespace controller


namespace operation {
    
    class Interpreter {
    public:
        Interpreter(controller::Knowledge* _kn);
        virtual ~Interpreter();

        bool parse(const char* const instruction,
                   bool* pRunopt, bool* pResetMotions);
        Eigen::VectorXd parseDir (const char* const dirname);
        int             parseNode(const char* const nodename);
        Eigen::VectorXd parsePose(const char* const posename);
    protected:
        MEMBER_PTR(controller::Knowledge*, kn);
    }; // class Interpreter
    
} // namespace operation

#endif // #ifndef OPERATION_INTERPRETER_H

