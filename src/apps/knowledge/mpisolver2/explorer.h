#ifndef MPISOLVER2_EXPLORER_H
#define MPISOLVER2_EXPLORER_H

#include "common/app_hppcommon.h"

#include "sample.h"

namespace controller {
    class SimPack;
} // namespace controller

namespace mpisolver2 {
    class CommandCenter;
    class Database;
} // namespace mpisolver2

// Forward declaration
namespace mpisolver2 {
    class Explorer;
    class NaiveExplorer;
    class DartThrowExplorer;
} // namespace mpisolver2

namespace mpisolver2 {

    class Explorer {
    public:
        Explorer(CommandCenter* _cmd);
        virtual ~Explorer();

        void incStartCount();
        void start();
        void stop();
        void mainloop();

        virtual Sample* generate() = 0;
        virtual int pickVictim() = 0;

    protected:
        int nChildProc();
        controller::SimPack* simpack();
        Database* db();
    protected:
        MEMBER_VAR(bool, isRunning);
        MEMBER_VAR(int, startCount);
        MEMBER_PTR(CommandCenter*, cmd);
    };

    class NaiveExplorer : public Explorer {
    public:
        NaiveExplorer(CommandCenter* _cmd);
        virtual ~NaiveExplorer();
        
        virtual Sample* generate();
        virtual int pickVictim();
    };
    
    class DartThrowExplorer : public Explorer {
    public:
        DartThrowExplorer(CommandCenter* _cmd);
        virtual ~DartThrowExplorer();

        virtual Sample* generate();

        virtual int pickVictim();
    protected:
        Sample* generateRandom();

    };

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_EXPLORER_H
