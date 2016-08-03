#ifndef MPISOLVER_CMAXMLWRITER_H
#define MPISOLVER_CMAXMLWRITER_H

#include <Eigen/Dense>
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"

#include "common/app_hppcommon.h"

class MPICMASearch;

namespace mpisolver {

    class CMAXMLWriter {
    public:
        CMAXMLWriter(MPICMASearch* _cma);
        virtual ~CMAXMLWriter();

        void allocate();
        void deallocate();

        bool addIteration(int iter);
        bool addResult();

        bool write(const char* const filename);

    protected:
        MEMBER_PTR(MPICMASearch*, cma);
        rapidxml::xml_document<> doc;
        rapidxml::xml_node<> *root;
    };

} // namespace mpisolver

#endif // #ifndef MPISOLVER_CMAXMLWRITER_H
