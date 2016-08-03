/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_BOUMANCLUSTER_H
#define SOLVER_BOUMANCLUSTER_H

#include <vector>
#include <Eigen/Dense>
#include "common/app_hppcommon.h"
#include "sample.h"

namespace solver {
    
    class BoumanCluster {
    public:
        BoumanCluster(std::vector<Sample*> _samples);
        virtual ~BoumanCluster();

        void cluster();
        void collapseByMeans();

        int numClasses() { return nClasses; }
        int getClass(int index) { return cls[index]; }
        std::vector<Sample*> getCluster(int index);

        static double g_clusterMinDist;
    protected:
        void run();
    protected:
        std::vector<Sample*> samples;
        int nClasses;
        std::vector<int> cls;
        std::vector<Eigen::VectorXd> clsMeans;
    };
    
} // namespace solver

#endif // #ifndef SOLVER_BOUMANCLUSTER_H

