#ifndef MPICLIENT_MPICLIENT_H
#define MPICLIENT_MPICLIENT_H

#include "common/app_hppcommon.h"

namespace mpiclient {

    class MPIClient {
    public:
        MPIClient();
        virtual ~MPIClient();

        virtual bool submitProblem() = 0;
        virtual bool executeSolver() = 0;
        virtual bool receiveStatus() = 0;
        virtual bool receiveResult() = 0;
        
    protected:

        virtual bool send(const char* const local, const char* remote) = 0;
        virtual bool recv(const char* const local, const char* remote) = 0;
        virtual bool execute(const char* const cmd) = 0;
    };

} // namespace mpiclient

#endif // #ifndef MPICLIENT_MPICLIENT_H
