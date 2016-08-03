#ifndef MPISOLVER2_EVALUATOR_H
#define MPISOLVER2_EVALUATOR_H

#include <vector>
#include <string>
#include "common/app_hppcommon.h"
#include "sample.h"

namespace controller {
    class SimPack;
} // namespace controller

namespace mpisolver2 {
    class CommandCenter;
    class Database;
} // namespace mpisolver2

namespace mpisolver2 {

    class Evaluator {
    public:
        Evaluator(CommandCenter* _cmd);
        virtual ~Evaluator();

        bool expandParams();
        void reevaluate();

    protected:
        int nChildProc();
        controller::SimPack* simpack();
        Database* db();
    protected:
        MEMBER_PTR(CommandCenter*, cmd);
        std::vector<std::string> param_names;
    };

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_EVALUATOR_H
