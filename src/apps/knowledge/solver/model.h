/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_MODEL_H
#define SOLVER_MODEL_H

#include <vector>
#include "common/app_hppcommon.h"
#include "sample.h"

class MyCMA;

namespace controller {
    class Knowledge;
} // namespace controller

namespace solver {
    struct ModelBezier;
    struct ModelPoint;
    struct ModelCombined;
    
    struct ModelBezier {
    public:
        ModelBezier();
        ModelBezier(int _dim);
        virtual ~ModelBezier();

        void initialize(int dim);
        double fit(const std::vector<Sample*>& samples);
        void update(controller::Knowledge* kn); // Update Model
        void updateModel(controller::Knowledge* kn); // Update Model
        Eigen::VectorXd predict(double t);

        int dim;
        Eigen::VectorXd alpha;
        Eigen::VectorXd beta;
        Eigen::MatrixXd C;
        double regressionError;

    }; // struct ModelBezier


    struct ModelPoint {
        ModelPoint(int _dim, controller::Knowledge* _kn);
        virtual ~ModelPoint();

        void init(const double _sigma,
                  const int _mu,
                  const std::vector<Sample*>& samples);
        void init(const double _sigma,
                  const int _mu,
                  const int _index,
                  const Eigen::VectorXd _m,
                  const std::vector<Sample*>& samples);
        void init(const double _sigma,
                  const int _mu,
                  const Eigen::VectorXd _m,
                  const Eigen::MatrixXd _C);
        static ModelPoint* interpolate(ModelPoint* lhs,
                                       ModelPoint* rhs,
                                       double w);

        Sample* create();
        void update(const std::vector<Sample*>& samples);
        void updateBestValues(const std::vector<Sample*>& samples, int index);
        Eigen::VectorXd diagonalOfCov();
        double volumn();
        double prob(const Eigen::VectorXd& x);
        double loglikelihood(const Eigen::VectorXd& x);

        int dim;
        double T;
        double sigma;
        int mu;

        double bestFitness;
        Eigen::VectorXd bestParams;

        Eigen::VectorXd m;
        Eigen::MatrixXd C;

        controller::Knowledge* kn;
        MyCMA* cma;

    }; // struct ModelPoint

    struct ModelCombined {
        ModelCombined(int _DIM,
                      const Eigen::VectorXd& _tasks,
                      controller::Knowledge* _kn);
        virtual ~ModelCombined();

        std::vector<double> bestValueArray();
        bool isActivated(int index) { return (activated[index] == 1); }
        void activate(int index) { activated[index] = 1; }
        int left(int index);
        int right(int index);
        int getNumActivatedCovs();

        int DIM;
        int loop;
        int NTASKS() { return tasks.size(); }
        Eigen::VectorXd tasks;
        controller::Knowledge* kn;
        ModelBezier* line;
        std::vector<ModelPoint*> covs;    // If activated, element is not null
        std::vector<Sample*> samples;
        std::vector<int> activated;
        bool isDegenerated;
    }; // struct ModelCombined;
    
} // namespace solver

#endif // #ifndef SOLVER_MODEL_H

