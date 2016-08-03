#ifndef MPISOLVER2_OPTIMIZER_H
#define MPISOLVER2_OPTIMIZER_H

#include <vector>
#include "common/app_hppcommon.h"
#include "sample.h"

namespace controller {
    class SimPack;
} // namespace controller

namespace mpisolver2 {
    class CommandCenter;
    class Database;
} // namespace mpisolver2

// Forward Declaration
class MPICMASearch;

namespace mpisolver2 {

    class Optimizer {
    public:
        Optimizer(CommandCenter* _cmd);
        virtual ~Optimizer();

        void deallocate();

        void start();
        void stop();
        void mainloop();


        std::string taskName();
    protected:
        int nChildProc();
        controller::SimPack* simpack();
        Database* db();
    protected:
        MEMBER_VAR(bool, isRunning);
        MEMBER_VAR(int, startCount);
        MEMBER_VAR(int, task);
        MEMBER_PTR(CommandCenter*, cmd);

        std::vector<MPICMASearch*> cmasearches;
    };


} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_OPTIMIZER_H
