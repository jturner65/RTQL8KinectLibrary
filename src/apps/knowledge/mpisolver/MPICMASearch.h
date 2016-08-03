#ifndef _MPICMA_H_
#define _MPICMA_H_

#include <mpi.h>

#include <SharkDefs.h>
#include <EALib/Population.h>
#include <EALib/PopulationT.h>
#include <EALib/SearchAlgorithm.h>
#include <Array/ArrayOp.h>
#include <Array/ArrayIo.h>
#include <LinAlg/LinAlg.h>
// #include <EALib/CMA.h>
#include "CMA2.h"
#include <EALib/ObjectiveFunctions.h>

#include <iostream>
#include <sstream>
#include <cstdio>

#include <Eigen/Dense>
#include "ClusterCMA.h"

#define BUFF_SIZE 512

class MPIFunc : public ObjectiveFunctionVS<double> {
public:
    MPIFunc(unsigned Dimension);
    unsigned int objectives() const;

    void result(double* const& point, std::vector<double>& value);
    double value(double* const& point);
    void sendMessage(int index, int numProcs, double* const& point);
    double recvMessage(int index, int numProcs, double* const& point);
};

//!
//! \brief Non-elitist CMA-ES implementing the interface EvolutionaryAlgorithm
//!
class MPICMASearch : public EvolutionaryAlgorithm<double*>
{
public:
    MPICMASearch();
    ~MPICMASearch();

    inline CMA2& getCMA2() { return m_cma; }
    inline const CMA2& getCMA2() const { return m_cma; }
    inline const PopulationT<double>* parents() const { return m_parents; }
    inline const PopulationT<double>* offspring() const { return m_offspring; }

    void init(ObjectiveFunctionVS<double>& fitness, CMA2::RecombType recomb = CMA2::superlinear, CMA2::UpdateType cupdate = CMA2::rankmu);
    void init(ObjectiveFunctionVS<double>& fitness, const Array<double>& start, double stepsize, CMA2::RecombType recomb = CMA2::superlinear, CMA2::UpdateType cupdate = CMA2::rankmu);
    void init(ObjectiveFunctionVS<double>& fitness, unsigned int mu, unsigned int lambda, const Array<double>& start, double stepsize, CMA2::RecombType recomb = CMA2::superlinear, CMA2::UpdateType cupdate = CMA2::rankmu);
    void init(ObjectiveFunctionVS<double>& fitness, unsigned int mu, unsigned int lambda, const Array<double>& start, const Array<double>& stepsize, CMA2::RecombType recomb = CMA2::superlinear, CMA2::UpdateType cupdate = CMA2::rankmu);
    void run(int numProcs, int iter);
    void kill(int numProcs);
    void bestSolutions(std::vector<double*>& points);
    void bestSolutionsFitness(Array<double>& fitness);

    friend std::ostream & operator<<( std::ostream & stream, const MPICMASearch& search);
    friend std::istream & operator>>( std::istream & stream, MPICMASearch& search);

    int numParents() { return m_parents->size(); }
    Eigen::VectorXd parent(int index);
    double parentValue(int index);

    int numOffsprings() { return m_offspring->size(); }
    Eigen::VectorXd offspring(int index);
    double offspringValue(int index);


    std::vector<int> clusterClass;
    std::vector<std::string> clusterName;
protected:
    CMA2 m_cma;
    ClusterCMA m_ccma;
    ObjectiveFunctionVS<double>* m_fitness;
    PopulationT<double>* m_parents;
    PopulationT<double>* m_offspring;
};
std::ostream & operator<<( std::ostream & stream, const MPICMASearch& search);
std::istream & operator>>( std::istream & stream, MPICMASearch& search);
#endif
