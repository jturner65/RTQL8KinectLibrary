#ifndef MPISOLVER2_WORKER_H
#define MPISOLVER2_WORKER_H

#include <string>
#include "common/app_hppcommon.h"

namespace controller {
    class SimPack;
} // namespace controller

namespace mpisolver2 {
    class Worker {
    public:
        Worker(int _rank, int _numProcs);
        virtual ~Worker();

        void initialize();
        void destroy();
        
        void mainloop();
        double simLoop();

        void onNewController();

        int numChildProcs() const { return numProcs() - 1; }


    protected:
        MEMBER_PTR(controller::SimPack*, simpack);

        MEMBER_VAR(int, rank);
        MEMBER_VAR(int, numProcs);
        std::string name;
    };

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_WORKER_H
