/*
 * C++ Interface for clust library written by Sehoon Ha.
 * Clust library: https://engineering.purdue.edu/~bouman/software/cluster/
 */

#ifndef MPISOLVER2_BOUMANCLUSTER_H
#define MPISOLVER2_BOUMANCLUSTER_H

#include <vector>
#include <Eigen/Dense>
#include "common/app_hppcommon.h"
#include "sample.h"

namespace mpisolver2 {
    class CommandCenter;
    class Database;
} // namespace mpisolver2

namespace mpisolver2 {
    class BoumanCluster {
    public:
        BoumanCluster(CommandCenter* _cmd);
        virtual ~BoumanCluster();

        void cluster();

        // int majorClass();
        int numClasses();
        // int getClass(int index);
        Eigen::VectorXd classMean(int ci) { return means[ci]; }
        Eigen::MatrixXd classCov(int ci)  { return covs[ci]; }
        Sample* bestSampleInClass(int ci);

    protected:
        void selectByFixedNumber();
        void selectByRatio();
        void run();
        Eigen::VectorXd expand(const Eigen::VectorXd& v, const std::vector<bool>& explored);
        Eigen::MatrixXd expand(const Eigen::MatrixXd& M, const std::vector<bool>& explored);
    protected:
        int nChildProc();
        Database* db();
    protected:
        MEMBER_PTR(CommandCenter*, cmd);
        std::vector<int> cls;
        std::vector<Sample*> goodsamples;
        std::vector<Eigen::VectorXd> means;
        std::vector<Eigen::MatrixXd> covs;
        
    };

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_BOUMANCLUSTER_H
