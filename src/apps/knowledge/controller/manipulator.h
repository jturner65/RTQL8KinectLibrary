/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_MANIPULATOR_H
#define controller_MANIPULATOR_H

#include "common/app_hppcommon.h"

namespace controller {
    class Knowledge;
} // namespace controller


namespace controller {
    
    class Manipulator {
    public:
        Manipulator(Knowledge* _kn);
        virtual ~Manipulator();

        
    protected:
        MEMBER_PTR(Knowledge*, kn);
    }; // class Manipulator
    
} // namespace controller

#endif // #ifndef controller_MANIPULATOR_H

