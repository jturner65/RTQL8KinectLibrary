#include "CMA2.h"
#include "boost/format.hpp"
#include "common/app_cppcommon.h"


int CMA2::g_cma2_id = 0;
void CMA2::resetID() {
    g_cma2_id = 0;
}


///////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
CMA2::CMA2()
    : parent(NULL)
{
    name = (boost::format("%c") % ((char)('A' + g_cma2_id))).str();
    ++g_cma2_id;
}

CMA2::CMA2(const CMA2& cma)
    : parent(NULL)
{
    n = cma.n;

    sigma = cma.sigma;
    chi_n = cma.chi_n;
    cc = cma.cc;
    cs = cma.cs;
    csu = cma.csu;
    ccu = cma.ccu;
    ccov = cma.ccov;
    d = cma.d;
    mueff = cma.mueff;
    mucov = cma.mucov;

    x = cma.x;
    xPrime = cma.xPrime;
    meanz = cma.meanz;

    z = cma.z;
    pc = cma.pc;
    ps = cma.ps;
    C = cma.C;
    Z = cma.Z;
    lambda = cma.lambda;
    B = cma.B;
    w = cma.w;
    theVector = cma.theVector;

    childs.clear();
    indflags = 0;
    name = (boost::format("%c") % ((char)('A' + g_cma2_id))).str();
    ++g_cma2_id;
}


CMA2::~CMA2() {
    destroySubtree();
}

///////////////////////////////////////////////////////////////////////////////
// Tree related functions
void CMA2::destroySubtree() {
    LOG(INFO) << FUNCTION_NAME();
    FOREACH(CMA2* child, childs) {
        delete child;
    }
    childs.clear();
}

bool CMA2::addChild(CMA2* c) {
    if (c->parent != NULL) {
        LOG(ERROR) << FUNCTION_NAME() << " a child node already has its own parent!!";
        return false;
    }

    childs.push_back(c);
    c->parent = this;
    return true;
}

bool CMA2::isRoot() {
    return (parent == NULL);
}

bool CMA2::isLeaf() {
    return (childs.size() == 0);
}

void CMA2::collectLeafNodes(std::vector<CMA2*>& leafNodes) {
    if (isLeaf()) {
        leafNodes.push_back(this);
        return;
    }

    FOREACH(CMA2* child, childs) {
        child->collectLeafNodes(leafNodes);
    }
}

void CMA2::resetIndFlagsSubtree() {
    indflags = 0;
    FOREACH(CMA2* child, childs) {
        child->resetIndFlagsSubtree();
    }
}

void CMA2::setIndFlagsToRoot(int id) {
    indflags = ( indflags | ( (long long)1 << id ) );
    if (!isRoot()) {
        parent->setIndFlagsToRoot(id);
    }
}

void CMA2::selectParentsSubtree(long long parentflags) {
    indflags = ( indflags & parentflags);
    FOREACH(CMA2* child, childs) {
        child->selectParentsSubtree(parentflags);
    }
    
}

void CMA2::printIndFlags(int n, long long indflags) {
    for (int i = 0; i < n; i++) {
        if ( (indflags & ( (long long)1 << i) ) != 0) {
            cout << "O ";
        } else {
            cout << "x ";
        }
        if (i % 10 == 9) cout << ", ";
    }
    cout << endl;
}

void CMA2::printIndFlagsSubtree(int n, int depth) {
    for (int i = 0; i < depth; i++) cout << "\t";
    cout << name << " ";
    cout << "(" << nCluSize << ")";
    cout << " : ";
    CMA2::printIndFlags(n, indflags);
    FOREACH(CMA2* child, childs) {
        child->printIndFlagsSubtree(n, depth + 1);
    }
}

CMA2* CMA2::findTheMostBottomNode(long long query) {
    // Search my childs first
    FOREACH(CMA2* child, childs) {
        CMA2* ret = child->findTheMostBottomNode(query);
        if (ret != NULL) {
            return ret;
        }
    }

    // If indflags contains query
    if ( (query & indflags) == query ) {
        return this;
    } else {
        return NULL;
    }
}

void CMA2::setCluSizeSubtree(int cn) {
   FOREACH(CMA2* child, childs) {
       child->setCluSizeSubtree(cn);
   }
   this->nCluSize = cn;
   
}

void CMA2::updateCMA(const PopulationT<double> &offspring) {
    PopulationT<double> my;
    for (int i = 0; i < offspring.size(); i++) {
        if ( (indflags & ( (long long)1 << i) ) != 0) {
            my.append( offspring[i] );
        }
    }
    if (my.size() > 0) {
        cout << "CMA " << name << " is updated" << endl;
        this->updateStrategyParameters( my );
    }
}

void CMA2::updateCMASubtree(const PopulationT<double> &offspring) {
   FOREACH(CMA2* child, childs) {
       child->updateCMASubtree(offspring);
   }
   this->updateCMA(offspring);
}

///////////////////////////////////////////////////////////////////////////////
// Original CMA functions

// static
unsigned CMA2::suggestLambda(unsigned dimension) {
    unsigned lambda = unsigned(4. + floor(3. * log((double) dimension)));
    // heuristics for small search spaces
    if (lambda > dimension) lambda = dimension; // CI's golden rule :-)
    if (lambda < 5) lambda = 5; // Hansen & Ostermeier's lower bound
    return lambda;
}

// static
unsigned CMA2::suggestMu(unsigned lambda, RecombType recomb) {
    if (recomb == equal) return  unsigned(floor(lambda / 4.));
    return  unsigned(floor(lambda / 2.));
}

void CMA2::init(unsigned dimension,
                std::vector<double > var, double _sigma,
                Population &p,
                RecombType recomb,
                UpdateType cupdate) {
    unsigned int i, j;

    unsigned mu = p.size();

    n     = dimension;
    sigma = _sigma;

    w.resize(mu);
    x.resize(n);
    xPrime.resize(n);
    z.resize(n);
    pc.resize(n);
    ps.resize(n);
    Z.resize(n, n);
    C.resize(n, n);
    B.resize(n, n);
    lambda.resize(n);
    theVector.resize(n);
    meanz.resize(n);

    switch (recomb) {
    case equal:
        for (i = 0; i < mu; i++) w(i) = 1;
        break;
    case linear:
        for (i = 0; i < mu; i++) w(i) = mu - i;
        break;
    case superlinear:
        for (i = 0; i < mu; i++) w(i) = log(mu + 1.) - log(1. + i);
        break;
    }

    double wSum    = 0;
    double wSumSqr = 0;
    for (i = 0; i < mu; i++) {
        wSum += w(i);
        wSumSqr += Shark::sqr(w(i));
    }
    w /= wSum; // normalizing weights
    wSumSqr /= Shark::sqr(wSum);
    mueff   = 1 / wSumSqr;

    // step size control
    cs      = (mueff + 2.)/(n + mueff + 3.);
    d       = 1. + 2. * Shark::max(0., sqrt( (mueff-1.)/(n+1) ) - 1.) + cs;

    // covariance matrix adaptation
    mucov   = mueff;
    if (cupdate == rankone) mucov = 1.;
    cc      = 4. / (4. + n);
    ccov    = 1. / mucov * 2. / Shark::sqr(n + sqrt(2.))
        + (1 - 1. / mucov) * Shark::min(1., (2 * mueff - 1) / (Shark::sqr(n + 2) + mueff));

    ccu     = sqrt((2. - cc) * cc);
    csu     = sqrt((2. - cs) * cs);
    chi_n   = sqrt(double(n)) * (1 - 1. / (4. * n) +  1. / (21. * Shark::sqr(double(n))));

    // init COG
    cog(x, p);

    // init paths
    for (i = 0; i < n; i++) {
        pc(i) = ps(i) = 0.;
        for (j = 0; j < n; j++) {
            if (i != j) C(i, j) = 0;
            else C(i, j) = var[i];
        }
    }

    // eigenvalues lambda and eigenvector matrix B
    eigensymm(C, B, lambda);
}

void CMA2::init(unsigned dimension, double _sigma,  Population &p,
                RecombType recomb, UpdateType cupdate) {
    std::vector<double> var(dimension);
    unsigned int i;
    for (i = 0; i < dimension; i++) var[i] = 1;
    init(dimension, var, _sigma, p, recomb, cupdate);
}

//
//! calculate weighted mean for intermediate recombination
//
void CMA2::cog(ChromosomeT<double >& a, Population &p, unsigned c) const {
    using namespace std;
    SIZE_CHECK(n == dynamic_cast<ChromosomeT<double >& >(p[0][c]).size());
    unsigned int i, j;
    for (j = 0; j < n; j++) {
        a[j] = dynamic_cast< ChromosomeT< double >& >(p[0][c])[j] * w(0);
    // if (c == 0) cout << (boost::format("a[%d] = %lf <- %lf * %lf") % j % a[j] % dynamic_cast< ChromosomeT< double >& >(p[0][c])[j] % w(0)) << endl;
        for (i = 1; i < p.size(); i++) {
            a[j] += dynamic_cast< ChromosomeT< double >& >(p[i][c])[j] * w(i);
            // if (c == 0) cout << (boost::format("a[%d] = %lf <- %lf * %lf") % j % a[j] % dynamic_cast< ChromosomeT< double >& >(p[i][c])[j] % w(i)) << endl;

        }
    // if (c == 0) cout << (boost::format("a[%d] = %lf") % j % a[j]) << endl;

    }
}

//! mutation after global intermediate recombination
//
void CMA2::create(Individual &o)
{
    unsigned int i, j;

    for (i = 0; i < n; i++) {
        // draw random vector
        z(i) = Rng::gauss(0, 1);
        dynamic_cast< ChromosomeT< double >& >(o[1])[i] = z(i);
        // global intermediate recombination, Eq. (1)
        dynamic_cast< ChromosomeT< double >& >(o[0])[i] = x[i];
    }

    // mutate objective variables, Eq. (1)
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            dynamic_cast< ChromosomeT< double >& >(o[0])[i] += sigma * B(i, j) * sqrt(fabs(lambda(j))) * z(j);
}

//
//! do the update of the covariance matrix and the global step size
//
void CMA2::updateStrategyParameters(Population &p, double lowerBound) {
    unsigned int i, j, k;
    double normPS = 0.;

    // COG of new parents
    cog(xPrime, p);
    cog(meanz , p, 1);

    theVector = 0;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            theVector(i) += B(i, j) * meanz[j];

    // Eq. (2) & Eq. (4)
    for (i = 0; i < n; i++) {
        pc(i) = (1 - cc) * pc(i) + ccu * sqrt(mueff) / sigma * (xPrime[i] - x[i]);
        ps(i) = (1 - cs) * ps(i) + csu * sqrt(mueff) * theVector(i);
        normPS += Shark::sqr(ps(i));
    }
    normPS  = sqrt(normPS);

    // Eq. (3)
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            for (Z(i, j) = 0., k = 0; k < p.size(); k++)
                Z(i, j) += w(k) * (dynamic_cast< ChromosomeT< double >& >(p[k][0])[i] - x[i]) *
                    (dynamic_cast< ChromosomeT< double >& >(p[k][0])[j] - x[j]);

    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            C(i, j) = (1 - ccov) * C(i, j) + ccov *
                (1. / mucov * pc(i) * pc(j) + (1. - 1. / mucov) * 1. / Shark::sqr(sigma) * Z(i, j));

    // Eq. (5)
    sigma *= exp((cs / d) * (normPS / chi_n - 1));

    // eigenvalues lambda and eigenvector matrix B
    eigensymm(C, B, lambda);

    // lower bound
    if ((sigma * sqrt(fabs(lambda(n - 1)))) < lowerBound)
        sigma = lowerBound / sqrt(fabs(lambda(n - 1)));

    // new COG becomes old COG
    x = xPrime;
}

//
//! get global setp size \f$\sigma\f$
//
double CMA2::getSigma() const {
    return sigma;
}

//
//! set global setp size \f$\sigma\f$
//
void CMA2::setSigma(double x) {
    sigma = x;
}

//
//! set different approximation for expectation of \f$\chi_n\f$ distribution
//
void CMA2::setChi_n(double x) {
    chi_n = x;
}

//
//! get condition number of covariance matrix
//
double CMA2::getCondition() const {
    return lambda(0) / lambda(n - 1);
}

//
//! get covariance matrix
//
const Array<double> &CMA2::getC() const {
    return C;
}

//
//! get eigenvalues of covariance matrix
//
const  Array<double> &CMA2::getLambda() const {
    return lambda;
}


CMA2 CMA2::duplicate() const {
    std::stringstream sout;
    sout << (*this);
    // std::cout << "sout.str() === " << sout.str() << std::endl;
    CMA2 dup;
    sout >> dup;
    return dup;
}

Eigen::VectorXd CMA2::center() {
    Eigen::VectorXd ret(n);
    for (int i = 0; i < n; i++) {
        ret(i) = x[i];
    }
    return ret;
}


std::string CMA2::toRstr() const {
    std::stringstream sout;
    for (int i = 0; i < n; i++) {
        if (i != 0) sout << ", ";
        sout << x[i];
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            sout << ", " << C(i, j);
        }
    }
    sout << ", " << name;
    sout << ", " << ( nCluSize > 0 ? 1 : 0);
    return sout.str();
}

std::string CMA2::toRstrSubtree() const {
    std::stringstream sout;
    sout << toRstr() << endl;
    FOREACH(CMA2* child, childs) {
        sout << child->toRstrSubtree() << endl;
    }
    return sout.str();
}

void CMA2::writeCSV(const char* const filename) const {
    using std::endl;
    std::ofstream fout(filename);
    fout << toRstr() << endl;
    fout.close();
    
}

std::ostream & operator<<( std::ostream & stream, const CMA2& cma)
{
    //change precision and save the old precision
    std::streamsize precision=stream.precision(21);

    //writing states
    stream<<cma.n<<" ";
    stream<<cma.sigma<<" ";
    stream<<cma.chi_n<<" ";
    stream<<cma.cc<<" ";
    stream<<cma.cs<<" ";
    stream<<cma.csu<<" ";
    stream<<cma.ccu<<" ";
    stream<<cma.ccov<<" ";
    stream<<cma.d<<" ";
    stream<<cma.mueff<<" ";
    stream<<cma.mucov<<" ";

    //reading chromosomes
    stream<<cma.x<<" ";
    stream<<cma.xPrime<<" ";
    stream<<cma.meanz<<" ";

    //reading arrays
    stream<<cma.z<<" ";
    stream<<cma.pc<<" ";
    stream<<cma.ps<<" ";
    stream<<cma.C<<" ";
    stream<<cma.Z<<" ";
    stream<<cma.lambda<<" ";
    stream<<cma.B<<" ";
    stream<<cma.w<<" ";
    stream<<cma.theVector<<" ";

    //reset precision
    stream.precision(precision);
    return stream;
}
std::istream & operator>>( std::istream & stream, CMA2& cma)
{
    //change precision and save the old precision
    double precision=stream.precision(21);
    //reading states
    stream>>cma.n;
    stream>>cma.sigma;
    stream>>cma.chi_n;
    stream>>cma.cc;
    stream>>cma.cs;
    stream>>cma.csu;
    stream>>cma.ccu;
    stream>>cma.ccov;
    stream>>cma.d;
    stream>>cma.mueff;
    stream>>cma.mucov;

    //reading chromosomes
    stream>>cma.x;
    stream>>cma.xPrime;
    stream>>cma.meanz;

    //reading arrays
    stream>>cma.z;
    stream>>cma.pc;
    stream>>cma.ps;
    stream>>cma.C;
    stream>>cma.Z;
    stream>>cma.lambda;
    stream>>cma.B;
    stream>>cma.w;
    stream>>cma.theVector;

    //reset precision state
    stream.precision(precision);
    return stream;
}

