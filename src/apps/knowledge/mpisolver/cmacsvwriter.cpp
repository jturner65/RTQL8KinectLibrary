#include "cmacsvwriter.h"

#include <fstream>
#include "boost/lexical_cast.hpp"
#include "common/app_cppcommon.h"
#include "MPICMASearch.h"

namespace mpisolver {
    CMACSVWriter::CMACSVWriter(MPICMASearch* _cma)
        : MEMBER_INIT(cma, _cma)
    {
        allocate();
    }

    CMACSVWriter::~CMACSVWriter() {
        deallocate();
    }

    void CMACSVWriter::allocate() {
        doc << "iter, id, x, y, value, type, cls, name" << endl;
    }
    
    void CMACSVWriter::deallocate() {
    }
    

    bool CMACSVWriter::addIteration(int iter) {
        for (int i = 0; i < cma()->numParents(); i++) {
            Eigen::VectorXd gene = cma()->parent(i);
            double value = cma()->parentValue(i);
            doc << iter;
            doc << ", " << i;
            doc << ", " << gene(0);
            doc << ", " << gene(1);
            doc << ", " << value;
            doc << ", " << "P";
            doc << ", " << cma()->clusterClass[i];
            doc << ", " << cma()->clusterName[i];
            doc << endl;
        }
        for (int i = 0; i < cma()->numOffsprings(); i++) {
            Eigen::VectorXd gene = cma()->offspring(i);

            bool same = false;
            for (int j = 0; j < cma()->numParents(); j++) {
                Eigen::VectorXd rhs = cma()->parent(j);
                if ( (gene - rhs).norm() < 0.001) {
                    same = true;
                    break;
                }
            }
            if (same) {
                continue;
            }

            double value = cma()->offspringValue(i);
            doc << iter;
            doc << ", " << i;
            doc << ", " << gene(0);
            doc << ", " << gene(1);
            doc << ", " << value;
            doc << ", " << "O";
            doc << ", " << -1;
            doc << ", " << "NG";
            doc << endl;
        }
        return true;
    }
    
    bool CMACSVWriter::addResult() {
        return true;
    }

    bool CMACSVWriter::write(const char* const filename) {
        std::ofstream fout(filename);
        fout << doc.str();
        fout.close();
    }


} // namespace mpisolver
