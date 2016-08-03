#ifndef MPISOLVER_CMACSVWRITER_H
#define MPISOLVER_CMACSVWRITER_H

#include <sstream>
#include "common/app_hppcommon.h"

class MPICMASearch;

namespace mpisolver {

    class CMACSVWriter {
    public:
        CMACSVWriter(MPICMASearch* _cma);
        virtual ~CMACSVWriter();

        void allocate();
        void deallocate();

        bool addIteration(int iter);
        bool addResult();

        bool write(const char* const filename);

    protected:
        MEMBER_PTR(MPICMASearch*, cma);
        std::stringstream doc;
    };


} // namespace mpisolver

#endif // #ifndef MPISOLVER_CMACSVWRITER_H
