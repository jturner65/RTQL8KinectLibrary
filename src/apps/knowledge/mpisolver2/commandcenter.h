#ifndef MPISOLVER2_COMMANDCENTER_H
#define MPISOLVER2_COMMANDCENTER_H

#include <string>
#include "common/app_hppcommon.h"

// To read instructions from file
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"

namespace controller {
    class SimPack;
} // namespace controller


namespace mpisolver2 {
    class Database;
    class Explorer;
    class Evaluator;
    class Optimizer;
};

namespace mpisolver2 {

    class CommandCenter {
    public:
        CommandCenter(int _nProc, bool _isLocal);
        virtual ~CommandCenter();

        bool mainloop();
        int nChildProc() { return nProc() - 1; }
        int dim();
    protected:
        void quit();
        std::string readCommand();
        bool writeStatus(const char* const status);
        void onNewInstruction(rapidxml::xml_node<>* root);

        MEMBER_PTR(controller::SimPack*, simpack);

        MEMBER_VAR(int, nProc);
        MEMBER_VAR(bool, isPaused);
        MEMBER_VAR(bool, isLocal);

        MEMBER_PTR(Database*, database);
        MEMBER_PTR(Explorer*, explorer);
        MEMBER_PTR(Evaluator*, evaluator);
        MEMBER_PTR(Optimizer*, optimizer);
    };

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_COMMANDCENTER_H
