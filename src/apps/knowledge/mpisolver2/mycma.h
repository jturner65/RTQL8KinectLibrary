#ifndef _MyCMA_H_
#define _MyCMA_H_

#include <SharkDefs.h>
#include <EALib/Population.h>
#include <EALib/PopulationT.h>
#include <EALib/SearchAlgorithm.h>
#include <Array/ArrayOp.h>
#include <Array/ArrayIo.h>
#include <LinAlg/LinAlg.h>

#include <iostream>

#include <Eigen/Dense>
#include "common/app_hppcommon.h"

//!
//! \brief Implements the most recent version of the non-elitist MyCMA-ES
//!
class MyCMA
{
public:
    MyCMA() {}

    virtual ~MyCMA() {}

    typedef enum { equal, linear, superlinear } RecombType;
    typedef enum { rankone, rankmu }            UpdateType;

    static unsigned suggestLambda(unsigned dimension);
    static unsigned suggestMu(unsigned lambda, RecombType recomb = superlinear);
    void init(unsigned dimension,
              double _sigma,
              Population &p,
              const Eigen::VectorXd& mean,
              const Eigen::MatrixXd& cov,
              RecombType recomb  = superlinear,
              UpdateType cupdate = rankmu);
    // void init(unsigned dimension,
    //           std::vector<double > var, double _sigma,
    //           Population &p,
    //           RecombType recomb  = superlinear,
    //           UpdateType cupdate = rankmu);
    // void init(unsigned dimension, double _sigma,  Population &p,
    //           RecombType recomb  = superlinear, UpdateType cupdate = rankmu);
    //
    // calculate weighted mean
    //
    void cog(ChromosomeT<double >& a, Population &p, unsigned c = 0) const;

    //
    // mutation after global intermediate recombination
    //
    void create(Individual &o);

    //
    // do the MyCMA
    //
    void updateStrategyParameters(Population &p, double lowerBound = .0);

    double getSigma() const;
    void   setSigma(double x);
    void   setChi_n(double x);
    double getCondition() const;
    const  Array<double> &getC() const;
    const  Array<double> &getLambda() const;


    friend std::ostream & operator<<( std::ostream & stream, const MyCMA& cma);
    friend std::istream & operator>>( std::istream & stream, MyCMA& cma);

protected:
    unsigned n;
    double sigma;
    double chi_n;
    double cc;
    double cs;
    double csu;
    double ccu;
    double ccov;
    double d;
    double mueff;
    double mucov;

    ChromosomeT<double> x;
    ChromosomeT<double> xPrime;
    ChromosomeT<double> meanz;

    Array<double> z;
    Array<double> pc;
    Array<double> ps;
    Array<double> C;
    Array<double> Z;
    Array<double> lambda;
    Array<double> B;
    Array<double> w;
    Array<double> theVector;
};

std::ostream & operator<<( std::ostream & stream, const MyCMA& cma);
std::istream & operator>>( std::istream & stream, MyCMA& cma);

#endif
