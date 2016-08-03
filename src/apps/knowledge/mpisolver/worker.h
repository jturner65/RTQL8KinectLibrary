#ifndef MPISOLVER_WORKER_H
#define MPISOLVER_WORKER_H

#include <string>
#include "common/app_hppcommon.h"

namespace rtql8 {
    namespace toolkit {
        struct SimReplay;
        class Simulator;
    } // namespace toolkit
} // namespace rtql8

namespace controller {
    class RootController;
} // namespace controller


namespace mpisolver {
    class Worker {
    public:
        Worker(int _rank, int _numProcs);
        virtual ~Worker();

        void initialize();
        void destroy();
        
        void mainloop();

        double simLoop();
        void simStep();

        int numChildProcs() const { return numProcs() - 1; }


    protected:
        MEMBER_PTR(rtql8::toolkit::Simulator*, sim);
        MEMBER_PTR(controller::RootController*, con);

        MEMBER_VAR(int, rank);
        MEMBER_VAR(int, numProcs);
        std::string name;
    };

} // namespace mpisolver

#endif // #ifndef MPISOLVER_WORKER_H
