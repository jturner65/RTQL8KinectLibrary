#include "MPICMASearch.h"
#include "common/app_cppcommon.h"

#include "commandcenter.h"
#include "database.h"
#include "optimizer.h"


////////////////////////////////////////////////////////////////////////////////
// class MPIFunc 
// MPIFunc::MPIFunc(unsigned Dimension)
MPIFunc::MPIFunc(mpisolver2::Optimizer* _opt, int dim)
    : MEMBER_INIT(opt, _opt)
    , ObjectiveFunctionVS<double>(dim, new BoxConstraintHandler(dim, -1, 1))
{
    m_name = "MPIFunc";
}

unsigned int MPIFunc::objectives() const {
    return 1;
}

void MPIFunc::result(double* const& point, std::vector<double>& value) {
    ++m_timesCalled;
}

double MPIFunc::value(double* const& point) {
    using namespace std;
    std::vector<double> value(1);
    result(point, value);
    return value[0];
}

void MPIFunc::sendMessage(int index, int nChildProc, double* const& point) {
    int child = (index % nChildProc) + 1;
    int dest = child;
    int tag = child;

    std::stringstream sout;
    for (int i = 0; i < dimension(); i++) {
        if (i != 0) sout << " ";
        sout << point[i];
    }

    char buff[MAX_PSM_BUFF_SIZE];
    sprintf(buff, "%s", sout.str().c_str());
    MPI_Send(&buff, MAX_PSM_BUFF_SIZE, MPI_PACKED, dest, tag, MPI_COMM_WORLD);
}

double MPIFunc::recvMessage(int index, int nChildProc, double* const& point) {
    using namespace std;
    MPI_Status status;

    int id = (index % nChildProc) + 1;
    const std::string NAME = (
        boost::format("[EXPThread %d/%d]") % id % nChildProc
        ).str();
    // LOG(INFO) << NAME << " waiting for response";

    double result;
    MPI_Recv(&result, 1, MPI_DOUBLE, id, id, MPI_COMM_WORLD, &status);
    // LOG(INFO) << NAME << " result = " << result;


    char buff[MAX_PSM_BUFF_SIZE];
    MPI_Recv(buff, MAX_PSM_BUFF_SIZE, MPI_PACKED, id, id, MPI_COMM_WORLD, &status);
    // LOG(INFO) << NAME << " finalstate recv = " << strlen(buff) << endl;


    ////////////////////////////////////////////////////////////
    // Create a sample
    // mpisolver2::Sample* s = new mpisolver2::Sample();
    // s->params = Eigen::VectorXd::Zero( dimension() );
    // for (int i = 0; i < dimension(); i++) {
    //     s->params(i) = point[i];
    // }

    Eigen::VectorXd params = Eigen::VectorXd::Zero( dimension() );
    for (int i = 0; i < dimension(); i++) {
        params(i) = point[i];
    }
    mpisolver2::Sample* s = new mpisolver2::Sample( params );

    s->value = result;
    s->prevValue = s->value;
    s->finalstate.fromString(buff);
    s->tag = (boost::format("OPT%02d%s") % opt()->startCount() % opt()->taskName() ).str();
    s->debugClusterId = opt()->task();
  opt()->cmd()->database()->add(s);
    ////////////////////////////////////////////////////////////
    

    return result;
    // std::vector<double> value(1);
    // result(point, value);
    // return value[0];
}
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// class MPICMASearch

MPICMASearch::MPICMASearch()
{
    m_name = "MPI-CMA-ES";

    m_parents = NULL;
    m_offspring = NULL;
}

MPICMASearch::~MPICMASearch()
{
    if (m_parents != NULL) delete m_parents;
    if (m_offspring != NULL) delete m_offspring;
}


// void MPICMASearch::init(ObjectiveFunctionVS<double>& fitness, MyCMA::RecombType recomb, MyCMA::UpdateType cupdate)
// {
//     unsigned int i, dim = fitness.dimension();

//     m_fitness = &fitness;
//     m_lambda = m_cma.suggestLambda(dim);
//     m_mu = m_cma.suggestMu(m_lambda, recomb);

//     // Sample three initial points and determine the
//     // initial step size as the median of their distances.
//     Vector start1(dim);
//     Vector start2(dim);
//     Vector start3(dim);
//     double* p;
//     p = &start1(0);
//     if (! fitness.ProposeStartingPoint(p)) throw SHARKEXCEPTION("[MPICMASearch::init] The fitness function must propose a starting point");
//     p = &start2(0);
//     if (! fitness.ProposeStartingPoint(p)) throw SHARKEXCEPTION("[MPICMASearch::init] The fitness function must propose a starting point");
//     p = &start3(0);
//     if (! fitness.ProposeStartingPoint(p)) throw SHARKEXCEPTION("[MPICMASearch::init] The fitness function must propose a starting point");
//     double d[3];
//     d[0] = (start2 - start1).norm();
//     d[1] = (start3 - start1).norm();
//     d[2] = (start3 - start2).norm();
//     std::sort(d, d + 3);
//     double stepsize = d[1]; if (stepsize == 0.0) stepsize = 1.0;

//     ChromosomeT<double> point(dim);
//     for (i=0; i<dim; i++) point[i] = start1(i);

//     m_parents = new PopulationT<double>(m_mu, point, ChromosomeT<double>(dim));
//     m_offspring = new PopulationT<double>(m_lambda, point, ChromosomeT<double>(dim));

//     m_parents->setMinimize();
//     m_offspring->setMinimize();

//     m_cma.init(dim, stepsize, *m_parents, recomb, cupdate);

//     // m_cma.nCluSize = m_mu;
//     // m_ccma.setCMA(m_cma);
// }

// void MPICMASearch::init(ObjectiveFunctionVS<double>& fitness, const Array<double>& start, double stepsize, MyCMA::RecombType recomb, MyCMA::UpdateType cupdate)
// {
//     unsigned int i, dim = fitness.dimension();
//     SIZE_CHECK(start.ndim() == 1);
//     SIZE_CHECK(start.dim(0) == dim);

//     m_fitness = &fitness;
//     m_lambda = m_cma.suggestLambda(dim);
//     m_mu = m_cma.suggestMu(m_lambda, recomb);

//     ChromosomeT<double> point(dim);
//     for (i=0; i<dim; i++) point[i] = start(i);

//     m_parents = new PopulationT<double>(m_mu, point, ChromosomeT<double>(dim));
//     m_offspring = new PopulationT<double>(m_lambda, point, ChromosomeT<double>(dim));

//     m_parents->setMinimize();
//     m_offspring->setMinimize();

//     m_cma.init(dim, stepsize, *m_parents, recomb, cupdate);
//     // m_cma.nCluSize = m_mu;
//     // m_ccma.setCMA(m_cma);
// }

void MPICMASearch::init(ObjectiveFunctionVS<double>& fitness,
                        unsigned int mu,
                        unsigned int lambda,
                        const Array<double>& start,
                        double stepsize,
                        const Eigen::VectorXd& mean,
                        const Eigen::MatrixXd& cov,
                        MyCMA::RecombType recomb,
                        MyCMA::UpdateType cupdate) {

    unsigned int i, dim = fitness.dimension();
    SIZE_CHECK(start.ndim() == 1);
    SIZE_CHECK(start.dim(0) == dim);

    m_fitness = &fitness;
    m_lambda = lambda;
    m_mu = mu;

    ChromosomeT<double> point(dim);
    for (i=0; i<dim; i++) point[i] = start(i);

    m_parents = new PopulationT<double>(m_mu, point, ChromosomeT<double>(dim));
    m_offspring = new PopulationT<double>(m_lambda, point, ChromosomeT<double>(dim));

    m_parents->setMinimize();
    m_offspring->setMinimize();

    m_cma.init(dim, stepsize, *m_parents, mean, cov, recomb, cupdate);

}

// void MPICMASearch::init(ObjectiveFunctionVS<double>& fitness, unsigned int mu, unsigned int lambda, const Array<double>& start, const Array<double>& stepsize, MyCMA::RecombType recomb, MyCMA::UpdateType cupdate)
// {
//     unsigned int i, dim = fitness.dimension();
//     SIZE_CHECK(start.ndim() == 1);
//     SIZE_CHECK(stepsize.ndim() == 1);
//     SIZE_CHECK(start.dim(0) == dim);
//     SIZE_CHECK(stepsize.dim(0) == dim);

//     m_fitness = &fitness;
//     m_lambda = lambda;
//     m_mu = mu;

//     ChromosomeT<double> point(dim);
//     for (i=0; i<dim; i++) point[i] = start(i);

//     m_parents = new PopulationT<double>(m_mu, point, ChromosomeT<double>(dim));
//     m_offspring = new PopulationT<double>(m_lambda, point, ChromosomeT<double>(dim));

//     m_parents->setMinimize();
//     m_offspring->setMinimize();

//     std::vector<double> var(dim);
//     for (i=0; i<dim; i++) var[i] = stepsize(i) * stepsize(i);
//     m_cma.init(dim, var, 1.0, *m_parents, recomb, cupdate);
//     // m_cma.nCluSize = m_mu;
//     // m_ccma.setCMA(m_cma);
// }

void MPICMASearch::run(int nChildProc, int iter)
{
    LOG(INFO) << FUNCTION_NAME() << " : begins";
    // m_ccma.calculateClusterIndex( m_offspring->size() );
    // cout << FUNCTION_NAME() << " : calculateClusterIndex OK" << endl;
    
    // printf("run [%d]\n", numProcs);
    int nSize = m_offspring->size();

    for (int k = 0; k < m_offspring->size(); k += nChildProc) {
        int upper = std::min(nSize, k + nChildProc);
        

        for (unsigned int i = k; i < upper; i++) {
            while (true) {
                m_cma.create((*m_offspring)[i]);
                if (m_fitness->isFeasible((*m_offspring)[i][0])) break;
            }
            ((MPIFunc*)m_fitness)->sendMessage(i, nChildProc, &(*m_offspring)[i][0][0]);

        }
        LOG(INFO) << "All messages sent. "<< k << " to " << upper;
        // printf("All messages sent. k = %d. # offsprings = %d\n", k, nSize);

        for (unsigned int i = k; i < upper; i++) {
            // for (unsigned int i = 0; i < m_offspring->size(); i++) {
            double f = ((MPIFunc*)m_fitness)->recvMessage(i, nChildProc, &(*m_offspring)[i][0][0]);
            (*m_offspring)[i].setFitness(f);

        }
        LOG(INFO) << "All responses received. "<< k << " to " << upper;
        // printf("All responses received. k = %d. # offsprings = %d\n", k, nSize);
    }
    

    m_parents->selectMuLambda(*m_offspring, 0u);
    m_cma.updateStrategyParameters(*m_parents);
    

    SearchAlgorithm<double*>::run();

    for (unsigned int i = 0; i < nSize; i++) {
        std::stringstream sout;
        for (int j = 0; j < (*m_offspring)[i][0].size(); j++) {
            if (j != 0) sout << " ";
            sout << (*m_offspring)[i][0][j];
        }
        LOG(INFO) << i << " : " << (*m_offspring)[i].fitnessValue() << " <- " << sout.str();
    }

}

// void MPICMASearch::kill(int numProcs) {
//     int numChildProcs = numProcs - 1;
//     char buff[BUFF_SIZE];
//     sprintf(buff, "QUIT");
//     for (int i = 0; i < numChildProcs; ++i)
//     {
//         int child = (i % numChildProcs) + 1;
//         int dest = child;
//         int tag = child;
//         MPI_Send(&buff, BUFF_SIZE, MPI_PACKED, dest, tag, MPI_COMM_WORLD);
//     }    
//     // printf("All QUIT messages sent.\n");
//     LOG(INFO) << "All QUIT messages sent.";
// }

void MPICMASearch::bestSolutions(std::vector<double*>& points)
{
    points.resize(1);
    points[0] = &((*m_parents)[0][0][0]);
}

void MPICMASearch::bestSolutionsFitness(Array<double>& fitness)
{
    fitness.resize(1, 1, false);
    fitness(0, 0) = (*m_parents)[0].fitnessValue();
}

Eigen::VectorXd MPICMASearch::parent(int index) {
    Eigen::VectorXd ret( (*m_parents)[index][0].size() );
    for (int j = 0; j < (*m_parents)[index][0].size(); j++) {
        ret(j) = (*m_parents)[index][0][j];
    }
    return ret;
}

double MPICMASearch::parentValue(int index) {
    return (*m_parents)[index].fitnessValue();
}


Eigen::VectorXd MPICMASearch::offspring(int index) {
    Eigen::VectorXd ret( (*m_offspring)[index][0].size() );
    for (int j = 0; j < (*m_offspring)[index][0].size(); j++) {
        ret(j) = (*m_offspring)[index][0][j];
    }
    return ret;
}

double MPICMASearch::offspringValue(int index) {
    return (*m_offspring)[index].fitnessValue();
}

std::ostream & operator<<( std::ostream & stream, const MPICMASearch& search)
{
    double precision=stream.precision(21);

    stream<<search.m_cma<<" "<<*search.m_parents<<" "<<*search.m_offspring<<std::flush;

    stream.precision(precision);
    return stream;
}
std::istream & operator>>( std::istream & stream, MPICMASearch& search)
{
    double precision=stream.precision(21);

    if(!search.m_parents)
        search.m_parents=new PopulationT<double>();
    if(!search.m_offspring)
        search.m_offspring=new PopulationT<double>();

    stream>>search.m_cma
          >>*search.m_parents
          >>*search.m_offspring;

    stream.precision(precision);
    return stream;
}
