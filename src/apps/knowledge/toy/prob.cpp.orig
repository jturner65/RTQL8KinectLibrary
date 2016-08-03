/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "prob.h"
#include "common/app_cppcommon.h"
#include "utils/Paths.h"
#include "utils/EigenHelper.h"
#include "utils/UtilsMath.h"

#include "controller/app_composite_controller.h"
#include "controller/knowledge.h"
#include "controller/simpack.h"

namespace toy {
    Sample::Sample() {
    }

    Sample::Sample(double _p, double _q) {
        params = Eigen::VectorXd::Zero(2);
        params << _p, _q;
    }


    std::ostream& operator << (std::ostream& sout, const Sample& s) {
        using boost::format;
        sout << "[";
        for (int i = 0; i < s.params.size(); i++) {
            sout << format("% .4lf ") % s.params(i);
        }
        sout << format("-> % .4lf % .4lf]") % s.task % s.error;
        return sout;
    }
    
    bool operator < (const Sample& lhs, const Sample& rhs) {
        return (lhs.error < rhs.error);
    }

    Model::Model(int _dim)
        : dim(_dim)
    {
        alpha = Eigen::VectorXd::Zero(dim);
        beta  = Eigen::VectorXd::Zero(dim);
        C     = Eigen::MatrixXd::Identity(dim, dim);
    }

    Eigen::VectorXd Model::randomFromCov() {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(C);
        if (eigensolver.info() != Eigen::Success) {
            cerr << "Cannot do eigen decomposition" << endl;
            exit(2);
        }
        Eigen::MatrixXd D = eigensolver.eigenvalues().asDiagonal();
        D = D.cwiseSqrt();
        Eigen::MatrixXd B = eigensolver.eigenvectors();
        return B * D * Eigen::VectorXd::Random(dim);
    }


    Sample Model::generate() {
        Sample s;
        // double t = rtql8::utils::random(1.0, 2.0);
        double t = rtql8::utils::random(MIN_TASK, MAX_TASK);
        Eigen::VectorXd CC = randomFromCov();

        s.params = alpha + t * beta + CC;
        for (int i = 0; i < dim; i++) {
            // s.params(i) = std::max(-1.0, std::min(s.params(i), 1.0));
            s.params(i) = std::max(MIN_PARAM, std::min(s.params(i), MAX_PARAM));
        }
        return s;

    }

    std::ostream& operator << (std::ostream& sout, const Model& m) {
        sout << "== Model ==" << endl;
        sout << "alpha = " << m.alpha.transpose() << endl;
        sout << "beta  = " << m.beta.transpose() << endl;
        sout << "cov = " << endl << m.C << endl;
        return sout;
    }

    ModelBezier::ModelBezier(int _dim)
        : Model(_dim)
    {
        gamma = Eigen::VectorXd::Zero(dim);
        delta = Eigen::VectorXd::Zero(dim);
    }


    Sample ModelBezier::generate() {
        Sample s;
        // double t = rtql8::utils::random(1.0, 2.0);
        double t = rtql8::utils::random(MIN_TASK, MAX_TASK);
        Eigen::VectorXd CC = randomFromCov();

        double a = (1 - t) * (1 - t) * (1 - t);
        double b = 3.0 * (1 - t) * (1 - t) * t;
        double c = 3.0 * (1 - t) * t * t;
        double d = t * t * t;

        s.params = a * alpha + b * beta + c * gamma + d * delta + CC;

        for (int i = 0; i < dim; i++) {
            // s.params(i) = std::max(-1.0, std::min(s.params(i), 1.0));
            s.params(i) = std::max(MIN_PARAM, std::min(s.params(i), MAX_PARAM));
        }
        return s;
    }

    std::ostream& operator << (std::ostream& sout, const ModelBezier& m) {
        sout << "== ModelBezier ==" << endl;
        sout << "alpha = " << m.alpha.transpose() << endl;
        sout << "beta  = " << m.beta.transpose()  << endl;
        sout << "gamma = " << m.gamma.transpose() << endl;
        sout << "delta = " << m.delta.transpose() << endl;
        sout << "cov = " << endl << m.C << endl;
        return sout;
    }



////////////////////////////////////////////////////////////
// class Prob implementation
    Prob::Prob(int _dim)
        : dim(_dim)
    {
    }
    
    Prob::~Prob() {
    }

// class Prob ends
////////////////////////////////////////////////////////////
    
////////////////////////////////////////////////////////////
// class ProbDB implementation
    ProbDB::ProbDB(const char* const filename)
        : Prob(2)
    {
        load(filename);
    }
    
    ProbDB::~ProbDB() {
    }

    void ProbDB::load(const char* const filename) {
        std::ifstream fin(filename);
        if  (!fin.is_open()) {
            cerr << "cannot load data: " << filename << endl;
            exit(1);
        }

        const int m = 9; // Items per line
        std::string temp;
        for (int i = 0; i < m; i++) {
            fin >> temp;
        }

        for (int loop = 0; ; loop++) {
            Sample s;
            s.params = Eigen::VectorXd::Zero(2);
            double x, y, z;

            for (int i = 0; i < m; i++) {
                if (i != 0) { char ch; fin >> ch; }
                double v;
                fin >> v;
                switch(i) {
                case 4: s.params(0) = v; break;
                case 5: s.params(1) = v; break;
                case 6: x = v; break;
                case 7: y = v; break;
                case 8: z = v; break;
                }
            }
            if (fin.fail()) break;

            s.task = y;
            double yhat = std::max(1.0, std::min(y, 2.0));
            s.error = sq(yhat - y) + sq(x) + sq(z);
            data.push_back(s);
        }
        fin.close();
        cout << "data.size() = " << data.size() << endl;
    }

    bool ProbDB::query(Sample* ps) {
        double sum_w  = 0;
        double sum_wt = 0;
        double sum_we = 0;

        BOOST_FOREACH(const Sample& s, data) {
            double dist_sq = (s.params - ps->params).squaredNorm();
            if (dist_sq > 0.01) {
                continue;
            }
            double w = 1 / (0.000001 + dist_sq);

            sum_w  += w;
            sum_wt += w * s.task;
            sum_we += w * s.error;
        }

        ps->task = sum_wt / sum_w;
        ps->error = sum_we / sum_w;
    
        return true;
    }
    
// class ProbDB ends
////////////////////////////////////////////////////////////
    

    
////////////////////////////////////////////////////////////
// class ProbSim implementation
    ProbSim::ProbSim()
        : Prob(0)
        , MEMBER_INIT_NULL(kn)
    {
        set_kn( new controller::Knowledge() );
        bool result = kn()->loadXML(RTQL8_DATA_PATH"/knowledge/test.xml");

        dim = con()->dim();
        LOG_INFO << endl << kn()->toString();
        LOG_INFO << "result = " << result;
        LOG_INFO << "dim = " << dim;
    }
    
    ProbSim::~ProbSim() {
    }

    bool ProbSim::query(Sample* ps) {
        simpack()->reset();
        con()->setParams(ps->params);
        simpack()->simulate();

        con()->evaluateTaskAndError(&(ps->task), &(ps->error));
     
        return true;
    }

    controller::SimPack* ProbSim::simpack() {
        return kn()->simpack();
    }

    rtql8::toolkit::Simulator* ProbSim::sim() {
        return simpack()->sim();
    }

    controller::AppCompositeController* ProbSim::con() {
        return simpack()->con();
    
    }

// class ProbSim ends
////////////////////////////////////////////////////////////
    
} // namespace toy



