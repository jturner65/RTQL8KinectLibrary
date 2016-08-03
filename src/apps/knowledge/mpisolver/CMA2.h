#ifndef _CMA2_H_
#define _CMA2_H_


#include <SharkDefs.h>
#include <EALib/Population.h>
#include <EALib/PopulationT.h>
#include <EALib/SearchAlgorithm.h>
#include <Array/ArrayOp.h>
#include <Array/ArrayIo.h>
#include <LinAlg/LinAlg.h>

#include <iostream>
#include <sstream>
#include <fstream>

#include <Eigen/Dense>
#include "common/app_hppcommon.h"

class CMA2 {
public:
    CMA2();
    CMA2(const CMA2& cma);

    virtual ~CMA2();


    typedef enum { equal, linear, superlinear } RecombType;
    typedef enum { rankone, rankmu }            UpdateType;

    static unsigned suggestLambda(unsigned dimension);
    static unsigned suggestMu(unsigned lambda, RecombType recomb = superlinear);
    void init(unsigned dimension,
              std::vector<double > var, double _sigma,
              Population &p,
              RecombType recomb  = superlinear,
              UpdateType cupdate = rankmu);
    void init(unsigned dimension, double _sigma,  Population &p,
              RecombType recomb  = superlinear, UpdateType cupdate = rankmu);
    //
    // calculate weighted mean
    //
    void cog(ChromosomeT<double >& a, Population &p, unsigned c = 0) const;

    //
    // mutation after global intermediate recombination
    //
    void create(Individual &o);

    //
    // do the CMA2
    //
    void updateStrategyParameters(Population &p, double lowerBound = .0);

    double getSigma() const;
    void   setSigma(double x);
    void   setChi_n(double x);
    double getCondition() const;
    const  Array<double> &getC() const;
    const  Array<double> &getLambda() const;

    CMA2 duplicate() const;
    Eigen::VectorXd center();

    std::string toRstr() const;
    void writeCSV(const char* const filename) const;


    friend std::ostream & operator<<( std::ostream & stream, const CMA2& cma);
    friend std::istream & operator>>( std::istream & stream, CMA2& cma);


public:
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

public: // Tree related variables and functions
    static void resetID();
    static int g_cma2_id;

    std::string name;
    CMA2* parent;
    std::vector<CMA2*> childs;
    int nCluSize; // The number of clusters who belongs to here

    long long indflags;
    
    void destroySubtree();
    bool addChild(CMA2* c);
    bool isRoot();
    bool isLeaf();
    void collectLeafNodes(std::vector<CMA2*>& leafNodes);

    void resetIndFlagsSubtree();
    void setIndFlagsToRoot(int id);
    void selectParentsSubtree(long long parentflags);
    static void printIndFlags(int n, long long indflags);
    void printIndFlagsSubtree(int n, int depth = 0);
    CMA2* findTheMostBottomNode(long long query);
    void setCluSizeSubtree(int cn);
    void updateCMA(const PopulationT<double> &offspring);
    void updateCMASubtree(const PopulationT<double> &offspring);
    std::string toRstrSubtree() const;

public: // Variables for my own extension

};

std::ostream & operator<<( std::ostream & stream, const CMA2& cma);
std::istream & operator>>( std::istream & stream, CMA2& cma);

#endif
