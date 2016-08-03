#ifndef MPISOLVER_RCLUSTERING_H
#define MPISOLVER_RCLUSTERING_H

#include <vector>
#include "common/app_hppcommon.h"

class MPICMASearch;

namespace mpisolver {
    class Rclustering {
    public:
        Rclustering(MPICMASearch* _cma);
        virtual ~Rclustering();

        void cluster();
        
        void write();
        void run();
        void read();
        
        int majorGroup();
        int numGroups();
        std::vector<int> cls;
    protected:
        virtual bool execute(const char* const cmd);
        MEMBER_PTR(MPICMASearch*, cma);

    };

} // namespace mpisolver

#endif // #ifndef MPISOLVER_RCLUSTERING_H
