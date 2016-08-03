#include "mycma.h"
#include "common/app_cppcommon.h"

// static
unsigned MyCMA::suggestLambda(unsigned dimension) {
    unsigned lambda = unsigned(4. + floor(3. * log((double) dimension)));
    // heuristics for small search spaces
    if (lambda > dimension) lambda = dimension; // CI's golden rule :-)
    if (lambda < 5) lambda = 5; // Hansen & Ostermeier's lower bound
    return lambda;
}

// static
unsigned MyCMA::suggestMu(unsigned lambda, RecombType recomb) {
    if (recomb == equal) return  unsigned(floor(lambda / 4.));
    return  unsigned(floor(lambda / 2.));
}

void MyCMA::init(unsigned dimension,
                 double _sigma,
                 unsigned _mu,
                 // Population &p,
                 const Eigen::VectorXd& mean,
                 const Eigen::MatrixXd& cov,
                 RecombType recomb,
                 UpdateType cupdate) {
    unsigned int i, j;

    // unsigned mu = p.size();
    unsigned mu = _mu;

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
    // cog(x, p);

    // init paths
    for (i = 0; i < n; i++) {
        x[i] = mean(i);
        pc(i) = ps(i) = 0.;
        for (j = 0; j < n; j++) {
            C(i, j) = cov(i, j);
            // if (i != j) C(i, j) = 0;
            // else C(i, j) = var[i];

        }
    }

    // eigenvalues lambda and eigenvector matrix B
    eigensymm(C, B, lambda);

}

// void MyCMA::init(unsigned dimension,
//                  std::vector<double > var, double _sigma,
//                  Population &p,
//                  RecombType recomb,
//                  UpdateType cupdate) {
//     unsigned int i, j;

//     unsigned mu = p.size();

//     n     = dimension;
//     sigma = _sigma;

//     w.resize(mu);
//     x.resize(n);
//     xPrime.resize(n);
//     z.resize(n);
//     pc.resize(n);
//     ps.resize(n);
//     Z.resize(n, n);
//     C.resize(n, n);
//     B.resize(n, n);
//     lambda.resize(n);
//     theVector.resize(n);
//     meanz.resize(n);

//     switch (recomb) {
//     case equal:
//         for (i = 0; i < mu; i++) w(i) = 1;
//         break;
//     case linear:
//         for (i = 0; i < mu; i++) w(i) = mu - i;
//         break;
//     case superlinear:
//         for (i = 0; i < mu; i++) w(i) = log(mu + 1.) - log(1. + i);
//         break;
//     }

//     double wSum    = 0;
//     double wSumSqr = 0;
//     for (i = 0; i < mu; i++) {
//         wSum += w(i);
//         wSumSqr += Shark::sqr(w(i));
//     }
//     w /= wSum; // normalizing weights
//     wSumSqr /= Shark::sqr(wSum);
//     mueff   = 1 / wSumSqr;

//     // step size control
//     cs      = (mueff + 2.)/(n + mueff + 3.);
//     d       = 1. + 2. * Shark::max(0., sqrt( (mueff-1.)/(n+1) ) - 1.) + cs;

//     // covariance matrix adaptation
//     mucov   = mueff;
//     if (cupdate == rankone) mucov = 1.;
//     cc      = 4. / (4. + n);
//     ccov    = 1. / mucov * 2. / Shark::sqr(n + sqrt(2.))
//         + (1 - 1. / mucov) * Shark::min(1., (2 * mueff - 1) / (Shark::sqr(n + 2) + mueff));

//     ccu     = sqrt((2. - cc) * cc);
//     csu     = sqrt((2. - cs) * cs);
//     chi_n   = sqrt(double(n)) * (1 - 1. / (4. * n) +  1. / (21. * Shark::sqr(double(n))));

//     // init COG
//     cog(x, p);

//     // init paths
//     for (i = 0; i < n; i++) {
//         pc(i) = ps(i) = 0.;
//         for (j = 0; j < n; j++) {
//             if (i != j) C(i, j) = 0;
//             else C(i, j) = var[i];
//         }
//     }

//     // eigenvalues lambda and eigenvector matrix B
//     eigensymm(C, B, lambda);
// }

// void MyCMA::init(unsigned dimension, double _sigma,  Population &p,
//                  RecombType recomb, UpdateType cupdate) {
//     std::vector<double> var(dimension);
//     unsigned int i;
//     for (i = 0; i < dimension; i++) var[i] = 1;
//     init(dimension, var, _sigma, p, recomb, cupdate);
// }

//
//! calculate weighted mean for intermediate recombination
//
void MyCMA::cog(ChromosomeT<double >& a, Population &p, unsigned c) const {
    SIZE_CHECK(n == dynamic_cast<ChromosomeT<double >& >(p[0][c]).size());
    unsigned int i, j;
    for (j = 0; j < n; j++) {
        a[j] = dynamic_cast< ChromosomeT< double >& >(p[0][c])[j] * w(0);
        for (i = 1; i < p.size(); i++) {
            a[j] += dynamic_cast< ChromosomeT< double >& >(p[i][c])[j] * w(i);
        }
    }
}

//! mutation after global intermediate recombination
//
void MyCMA::create(Individual &o)
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

void MyCMA::create(Eigen::VectorXd& o) {
    unsigned int i, j;

    // cout << "MyCMA::create. n = " << n << endl;
    o = Eigen::VectorXd::Zero(n);

    for (i = 0; i < n; i++) {
        // draw random vector
        z(i) = Rng::gauss(0, 1);
        o(i) = x[i];
    }

    // mutate objective variables, Eq. (1)
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            o(i) += sigma * B(i, j) * sqrt(fabs(lambda(j))) * z(j);
}


//
//! do the update of the covariance matrix and the global step size
//
void MyCMA::updateWeight(int _mu, RecombType recomb, UpdateType cupdate) {
    unsigned int i, j;

    // unsigned mu = p.size();
    unsigned mu = _mu;

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
    
}

void MyCMA::updateStrategyParameters(Population &p, double lowerBound) {
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
double MyCMA::getSigma() const {
    return sigma;
}

//
//! set global setp size \f$\sigma\f$
//
void MyCMA::setSigma(double x) {
    sigma = x;
}

//
//! set different approximation for expectation of \f$\chi_n\f$ distribution
//
void MyCMA::setChi_n(double x) {
    chi_n = x;
}

//
//! get condition number of covariance matrix
//
double MyCMA::getCondition() const {
    return lambda(0) / lambda(n - 1);
}

//
//! get covariance matrix
//
const Array<double> &MyCMA::getC() const {
    return C;
}

//
//! get eigenvalues of covariance matrix
//
const  Array<double> &MyCMA::getLambda() const {
    return lambda;
}

Eigen::VectorXd MyCMA::getMean() {
    Eigen::VectorXd v(n);
    for (int i = 0; i < n; i++) {
        v(i) = x[i];
    }
    return v;
}

Eigen::MatrixXd MyCMA::getCov() {
    Eigen::MatrixXd m(n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            m(i, j) = C(i, j);
        }
    }
    return m;
}

std::ostream & operator<<( std::ostream & stream, const MyCMA& cma)
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
std::istream & operator>>( std::istream & stream, MyCMA& cma)
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


