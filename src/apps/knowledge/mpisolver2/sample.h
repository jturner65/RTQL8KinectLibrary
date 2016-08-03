#ifndef MPISOLVER2_SAMPLE_H
#define MPISOLVER2_SAMPLE_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "common/app_hppcommon.h"
#include "toolkit/SimStates.h"
#include "controller/phase.h"

#define MAX_PSM_BUFF_SIZE 4096

namespace mpisolver2 {

    struct Sample {
        Sample(const Eigen::VectorXd& _params);
        static int g_nextid;

        int dim() const { return params.size(); }
        Eigen::VectorXd isParamUsedAsEigen() const;

        Eigen::VectorXd reducedParam(const std::vector<bool>& use);

        // Member variables
        Eigen::VectorXd params;
        std::vector<bool> isParamUsed; // is this param used for simulation?
        controller::PhaseStateMap finalstate;
        double value;
        double prevValue;

        int id;
        bool isSimulated;
        std::string tag;

        int debugClusterId; // Not supposed to be used for code: just for debug

        // Manipulation functions
        void simulate();
        void evaluate();
        
    };

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_SAMPLE_H
