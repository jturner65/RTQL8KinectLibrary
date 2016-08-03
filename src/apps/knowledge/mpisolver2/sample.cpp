#include "sample.h"
#include "common/app_cppcommon.h"

namespace mpisolver2 {

    ////////////////////////////////////////////////////////////
    // struct Sample implementation
    int Sample::g_nextid = 0;

    Sample::Sample(const Eigen::VectorXd& _params)
        : isSimulated(false)
        , params(_params)
        , value(0.0)
        , prevValue(-1.0)
        , debugClusterId(-1)
    {
        this->id = g_nextid++;
        this->tag = "";

        isParamUsed.clear();
        for (int i = 0; i < dim(); i++) {
            isParamUsed.push_back(true);
        }
    }

    Eigen::VectorXd Sample::isParamUsedAsEigen() const {
        Eigen::VectorXd ret( dim() );
        for (int i = 0; i < dim(); i++) {
            ret(i) = ( isParamUsed[i] ) ? 1 : 0;
        }
        return ret;
    }

    Eigen::VectorXd Sample::reducedParam(const std::vector<bool>& use) {
        int cnt = 0;
        Eigen::VectorXd ret( dim() );
        for (int i = 0; i < dim(); i++) {
            if (use[i] == false) {
                continue;
            }
            
            ret(cnt++) = params(i);
        }
        return ret.head(cnt);
        
    }

    void Sample::simulate() {
    }
    
    void Sample::evaluate() {
    }
    

    // 
    ////////////////////////////////////////////////////////////
} // namespace mpisolver2

