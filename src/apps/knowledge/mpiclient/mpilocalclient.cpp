#include "mpilocalclient.h"

#include <cstdio>
#include "common/app_cppcommon.h"

namespace mpiclient {

    MPILocalClient::MPILocalClient(int _np)
        : MEMBER_INIT(np, _np)
    {
    }

    MPILocalClient::~MPILocalClient() {
    }

    bool MPILocalClient::submitProblem() {
        // Do nothing, probably
        LOG(INFO) << FUNCTION_NAME() << " OK : it's local, so do nothing";
        return true;
    }
    
    bool MPILocalClient::executeSolver() {
        LOG(INFO) << FUNCTION_NAME();
        bool ret =  execute("mpirun -np 4 ./mpisolver");
        LOG(INFO) << FUNCTION_NAME() << " OK";
        return ret;
    }
    
    bool MPILocalClient::receiveStatus() {
        LOG(INFO) << FUNCTION_NAME() << " OK";
        return true;
    }
    
    bool MPILocalClient::receiveResult() {
        LOG(INFO) << FUNCTION_NAME() << " OK : it's local, so do nothing";
        return true;
    }

    bool MPILocalClient::send(const char* const local, const char* remote) {
    }
    
    bool MPILocalClient::recv(const char* const local, const char* remote) {
    }
    
    bool MPILocalClient::execute(const char* const cmd) {
        FILE* pipe = popen(cmd, "r");
        if (!pipe) {
            return false;
        }
        
        char buffer[128];
        // std::string result = "";
        while(!feof(pipe)) {
            if(fgets(buffer, 128, pipe) != NULL) {
                cout << buffer << std::flush;
    		// result += buffer;
            }
        }
        pclose(pipe);
        return true;
    }

} // namespace mpiclient
