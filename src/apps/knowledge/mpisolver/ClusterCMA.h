#ifndef MPISOLVER_CLUSTERCMA_H
#define MPISOLVER_CLUSTERCMA_H

#include <SharkDefs.h>
#include <EALib/Population.h>
#include <EALib/PopulationT.h>
#include <EALib/IndividualT.h>
#include <EALib/SearchAlgorithm.h>
#include <Array/ArrayOp.h>
#include <Array/ArrayIo.h>
#include <LinAlg/LinAlg.h>

#include "common/app_hppcommon.h"
#include "CMA2.h"
#include "Rclustering.h"


class ClusterCMA {
public:
    ClusterCMA();
    virtual ~ClusterCMA();

    void setCMA(const CMA2& cma);

    void calculateClusterIndex(int mu);
    void create(Individual &o, int index);
    // Eigen::VectorXd toEigen(IndividualT<double>& ind);
    // Eigen::VectorXd mean(PopulationT<double> &p);
    // CMA2& findParent(PopulationT<double> &p);
    
    void updateStrategyParameters(mpisolver::Rclustering& clust,
                                  PopulationT<double> &p,
                                  const PopulationT<double> &offspring);
                                  
    std::string toRstr() const;
    void writeCSV(const char* const filename) const;

    std::vector<std::string> name;
protected:
    // CMA2 orig;
    CMA2* root; // Root of the clusters
    std::vector<CMA2*> indexToCluster;
protected:
    int findIndex(const IndividualT<double> &o, const PopulationT<double> &population);
    long long toIndflags(const PopulationT<double> &my, const PopulationT<double> &population);

    // std::vector<int> cluId;
    // std::vector<CMA2> CMAs;
    
};

#endif // #ifndef MPISOLVER_CLUSTERCMA_H
