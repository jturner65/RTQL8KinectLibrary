/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef TOY_PROB_H
#define TOY_PROB_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "utils/EigenHelper.h"
#include "common/app_hppcommon.h"

#ifndef SQ_DEFINED
#define SQ_DEFINED
#define sq(x) ((x) * (x))
#endif

#define MIN_TASK 0.0
#define MAX_TASK 1.0

#define MIN_PARAM -1.0
#define MAX_PARAM  1.0


namespace rtql8 {
    namespace toolkit {
        class Simulator;
    } // namespace toolkit
} // namespace rtql8


namespace controller {
    class SimPack;
    class Knowledge;
    class AppCompositeController;
} // namespace controller

namespace toy {
    struct Sample {
        Eigen::VectorXd params;
        double task;
        double error;

        Sample();
        Sample(double _p, double _q);
    };
    
    std::ostream& operator << (std::ostream& sout, const Sample& s);
    bool operator < (const Sample& lhs, const Sample& rhs);

    struct Model {
        int dim;
        Eigen::VectorXd alpha;
        Eigen::VectorXd beta;
        Eigen::MatrixXd C;

        Model(int _dim);

        Eigen::VectorXd randomFromCov();
        virtual Sample generate();
    };

    std::ostream& operator << (std::ostream& sout, const Model& m);

    struct ModelBezier : public Model {
        Eigen::VectorXd gamma;
        Eigen::VectorXd delta;
        ModelBezier(int _dim);
        virtual Sample generate();
    };

    std::ostream& operator << (std::ostream& sout, const ModelBezier& m);

    class Prob {
    public:
        Prob(int _dim);
        virtual ~Prob();

        virtual bool query(Sample* ps) = 0;

        int dim;
    protected:
        
    }; // class Prob

    class ProbDB : public Prob {
    public:
        ProbDB(const char* const filename);
        virtual ~ProbDB();

        void load(const char* const filename);
        virtual bool query(Sample* ps);
    protected:
        std::vector<Sample> data;
    }; // class ProbDB

    class ProbSim : public Prob {
    public:
        ProbSim();
        virtual ~ProbSim();

        virtual bool query(Sample* ps);
    protected:
        std::vector<Sample> data;
        MEMBER_PTR(controller::Knowledge*, kn);
        controller::SimPack* simpack();
        rtql8::toolkit::Simulator* sim();
        controller::AppCompositeController* con();
    }; // class ProbSim


    
} // namespace toy

#endif // #ifndef TOY_PROB_H

