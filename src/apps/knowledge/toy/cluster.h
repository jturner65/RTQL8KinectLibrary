/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef TOY_CLUSTER_H
#define TOY_CLUSTER_H

#include <fstream>
#include "common/app_hppcommon.h"
#include "prob.h"

namespace toy {
    
    class Cluster {
    public:
        typedef std::vector<toy::Sample> Set;
        Cluster();
        Cluster(std::vector<toy::Sample>& _samples);
        virtual ~Cluster();

        void test();

        void split_w_task(Set& input, double task_lo, double task_hi, Set& output);
        void split_w_cluster(Set& input, Set& output);
        void writeFileWithGroup(std::ostream& fout, Set& input, std::string group);

        Set merge(Set& lhs, Set& rhs);
        Set merge(Set& c0, Set& c1, Set& c2);


        Set getCluster(int index) { return clusters[index]; }
        
    protected:
        std::vector<toy::Sample> samples;
        std::vector<Eigen::VectorXd> means;
        std::vector<Eigen::MatrixXd> covs;

        std::vector<Set> clusters;

        

    }; // class Cluster
    
} // namespace toy

#endif // #ifndef TOY_CLUSTER_H

