#ifndef MPICLIENT_MPISSHCLIENT_H
#define MPICLIENT_MPISSHCLIENT_H

#include <string>
#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "mpiclient.h"

namespace mpiclient {

    class MPISSHClient {
    public:
        MPISSHClient();
        virtual ~MPISSHClient();

        
        virtual bool submitProblem();
        virtual bool executeSolver();
        virtual bool receiveStatus();
        virtual bool receiveResult();
        
    protected:
        void connect();

        virtual bool send(const char* const local, const char* remote);
        virtual std::string read(const char* remote);
        virtual bool recv(const char* const local, const char* remote);
        virtual bool execute(const char* const cmd);

        static int verify_knownhost(ssh_session session);
        ssh_session ssh;
        sftp_session sftp;
    };

} // namespace mpiclient

#endif // #ifndef MPICLIENT_MPISSHCLIENT_H
