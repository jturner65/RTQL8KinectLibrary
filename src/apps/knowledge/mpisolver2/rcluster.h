#ifndef MPISOLVER2_RCLUSTER_H
#define MPISOLVER2_RCLUSTER_H

#include <vector>
#include <Eigen/Dense>
#include "common/app_hppcommon.h"
#include "sample.h"

namespace controller {
    class SimPack;
} // namespace controller

namespace mpisolver2 {
    class CommandCenter;
    class Database;
} // namespace mpisolver2

namespace mpisolver2 {

    class Rcluster {
    public:
        Rcluster(CommandCenter* _cmd);
        virtual ~Rcluster();

        void cluster();

        int majorClass();
        int numClasses();
        int getClass(int index);
        Eigen::VectorXd classMean(int ci) { return means[ci]; }
        Eigen::MatrixXd classCov(int ci)  { return covs[ci]; }
    protected:
        void select();
        void selectByRatio();
        void write();
        void run();
        void read();
        Eigen::VectorXd readMean(int cls);
        Eigen::MatrixXd readCov(int cls);

    protected:
        int nChildProc();
        controller::SimPack* simpack();
        Database* db();
        virtual bool execute(const char* const cmd);

    protected:
        // MEMBER_VAR(bool, isRunning);
        MEMBER_PTR(CommandCenter*, cmd);
        std::vector<int> cls;
        std::vector<Sample*> goodsamples;
        std::vector<Eigen::VectorXd> means;
        std::vector<Eigen::MatrixXd> covs;
    };
    

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_RCLUSTER_H
