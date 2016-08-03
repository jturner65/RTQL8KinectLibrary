#ifndef MPICLIENT_MPILOCALCLIENT_H
#define MPICLIENT_MPILOCALCLIENT_H

#include "mpiclient.h"

namespace mpiclient {
    class MPILocalClient {
    public:
        MPILocalClient(int _np);
        virtual ~MPILocalClient();

        virtual bool submitProblem();
        virtual bool executeSolver();
        virtual bool receiveStatus();
        virtual bool receiveResult();
        
    protected:

        virtual bool send(const char* const local, const char* remote);
        virtual bool recv(const char* const local, const char* remote);
        virtual bool execute(const char* const cmd);

        MEMBER_VAR(int, np);
        
    };

} // namespace mpiclient

#endif // #ifndef MPICLIENT_MPILOCALCLIENT_H
