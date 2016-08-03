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
#include "mycma.h"
#include <EALib/ObjectiveFunctions.h>

#include <iostream>
#include <sstream>
#include <cstdio>

#include <Eigen/Dense>


namespace mpisolver2 {
    class CommandCenter;
    class Database;
    class Optimizer;
} // namespace mpisolver2

class MPIFunc : public ObjectiveFunctionVS<double> {
public:
    MPIFunc(mpisolver2::Optimizer* _opt, int dim);
    unsigned int objectives() const;

    void result(double* const& point, std::vector<double>& value);
    double value(double* const& point);
    void sendMessage(int index, int nChildProc, double* const& point);
    double recvMessage(int index, int nChildProc, double* const& point);
protected:
    MEMBER_PTR(mpisolver2::Optimizer*, opt);
};

//!
//! \brief Non-elitist CMA-ES implementing the interface EvolutionaryAlgorithm
//!
class MPICMASearch : public EvolutionaryAlgorithm<double*>
{
public:
    MPICMASearch();
    ~MPICMASearch();

    inline MyCMA& getCMA() { return m_cma; }
    inline const MyCMA& getCMA() const { return m_cma; }
    inline const PopulationT<double>* parents() const { return m_parents; }
    inline const PopulationT<double>* offspring() const { return m_offspring; }

    // void init(ObjectiveFunctionVS<double>& fitness, MyCMA::RecombType recomb = MyCMA::superlinear, MyCMA::UpdateType cupdate = MyCMA::rankmu);
    // void init(ObjectiveFunctionVS<double>& fitness, const Array<double>& start, double stepsize, MyCMA::RecombType recomb = MyCMA::superlinear, MyCMA::UpdateType cupdate = MyCMA::rankmu);
    void init(ObjectiveFunctionVS<double>& fitness,
              unsigned int mu,
              unsigned int lambda,
              const Array<double>& start,
              double stepsize,
              const Eigen::VectorXd& mean,
              const Eigen::MatrixXd& cov,
              MyCMA::RecombType recomb = MyCMA::superlinear,
              MyCMA::UpdateType cupdate = MyCMA::rankmu);
    // void init(ObjectiveFunctionVS<double>& fitness, unsigned int mu, unsigned int lambda, const Array<double>& start, const Array<double>& stepsize, MyCMA::RecombType recomb = MyCMA::superlinear, MyCMA::UpdateType cupdate = MyCMA::rankmu);
    void run(int nChildProc, int iter);
    // void kill(int numProcs);
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
    MyCMA m_cma;
    ObjectiveFunctionVS<double>* m_fitness;
    PopulationT<double>* m_parents;
    PopulationT<double>* m_offspring;
};
std::ostream & operator<<( std::ostream & stream, const MPICMASearch& search);
std::istream & operator>>( std::istream & stream, MPICMASearch& search);
#endif
