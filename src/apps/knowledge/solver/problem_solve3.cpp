/* Rtql8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include <SharkDefs.h>
#include <Array/ArrayOp.h>
#include <Array/ArrayIo.h>
#include <EALib/Population.h>
#include <EALib/PopulationT.h>
#include <EALib/SearchAlgorithm.h>
#include <LinAlg/LinAlg.h>
#include <EALib/CMA.h>
#include <EALib/ObjectiveFunctions.h>

#include "problem.h"

#include <algorithm>
#include <set>
#include <boost/scoped_ptr.hpp>

#include "utils/UtilsMath.h"

#include "common/app_cppcommon.h"

#include "controller/app_composite_controller.h"
#include "controller/knowledge.h"
#include "controller/simpack.h"

#include "solver.h"
#include "evaluator.h"
#include "explorer.h"
#include "model.h"
#include "boumancluster.h"


#define ACTIVATED   1
#define DEACTIVATED 0

namespace solver {

    class ObjectiveFunc : public ObjectiveFunctionVS<double> {
    public:
        ObjectiveFunc(int _DIM, Solver* _solver, controller::Knowledge* _kn) :
            ObjectiveFunctionVS<double>(_DIM),
            DIM(_DIM),
            solver(_solver),
            kn(_kn)
            {}
        unsigned int objectives() const { return 1; }

        void result(double* const& point, std::vector<double>& value) {
            // Define variables
            int N = DIM / 2;
            Eigen::VectorXd alpha = Eigen::VectorXd::Zero(N);
            Eigen::VectorXd alpha2 = Eigen::VectorXd::Zero(N);
            Eigen::VectorXd beta  = Eigen::VectorXd::Zero(N);
            Eigen::VectorXd tasks(6);


            for (int i = 0;i < 3; i++) {
                Rng::uni(-1.0, 1.0);
            }

            for (int i = 0; i < N; i++) {
                alpha(i) = point[i];
                alpha2(i)  = point[i + N];
                beta(i) = alpha2(i) - alpha(i);
            }
            // LOG_INFO << "alpha, beta = " << IO(alpha) << " " << IO(beta);
            tasks << 0.0, 0.2, 0.4, 0.6, 0.8, 1.0;

            // Generate samples
            std::vector<Sample*> samples;
            for (double w = 0.0; w < 1.00001; w += 0.2) {
                // Eigen::VectorXd p = alpha + w * beta;
                Sample* s = new Sample(kn->simpack()->con());
                s->params = alpha + w * beta;
                s->evaluate(kn);
                s->reevaluate(kn, tasks);
                samples.push_back(s);
            }


            // // Evaluate
            // solver->evaluator()->evaluate( samples );
            // solver->evaluator()->reevaluate( samples, tasks , false);
            
            double sum = 0.0;
            for (int i = 0; i < samples.size(); i++) {
                Sample* s = samples[i];
                double v = s->tvalues(i);
                // LOG_INFO << i << " : " << IO(samples[i]->tvalues) << " --> " << v;
                sum += v;
                delete s;
            }
            double avg = sum / 6.0;
            LOG_INFO << g_count << " "
                     << "alpha, beta = " << IO(alpha) << " " << IO(beta) << " "
                     << "avg = " << avg;

            value.resize(1);
            value[0] = avg;
            g_count++;
        }
        static int g_count;
        int DIM;
        Solver* solver;
        controller::Knowledge* kn;
    };

    int ObjectiveFunc::g_count = 0;

    bool Problem::solve_simple(Solver* solver) {

        const std::string TAG = "[SIMPLE_SOLVER] ";
        controller::Knowledge* kn = solver->kn();
        std::vector<Sample*> samples = solver->getSamples();
        const int DIM  = samples[0]->params.size();
        const int DIM2 = DIM * 2;
        LOG_INFO << TAG << " begins. DIM = " << DIM;

	// Paraboloid f(DIM2, 1000);
	ObjectiveFunc f(DIM2, solver, kn);
        LOG_INFO << TAG << " function created";

	// start point
	const double MinInit        = -1.0;
	const double MaxInit        = 1.0;
	const double GlobalStepInit = 1.;
	::Array<double> start(DIM2);
	start = Rng::uni(MinInit, MaxInit);
        LOG_INFO << TAG << " start point found";

        // File Stream
        std::ofstream fout("baseline.csv");


	// search algorithm
	CMASearch cma;
        int mu = 8;
        int lambda = 16;
        cma.init(f, mu, lambda, start, GlobalStepInit);
	// cma.init(f, start, GlobalStepInit);

        for (int loop = 0; loop < 15; loop++) {
            ObjectiveFunc::g_count = 0;
            LOG_INFO << TAG << "**** loop = " << loop;
            solver->set_motionUpdatedIteration(loop);
            // Eigen::VectorXd alpha = Eigen::VectorXd::Random(DIM);
            // Eigen::VectorXd beta  = Eigen::VectorXd::Random(DIM);
            // LOG_INFO << TAG << "alpha, beta = " << IO(alpha) << " " << IO(beta);
            // kn->simpack()->con()->setRegressionParams( alpha, beta );

            // cma
            cma.run();
            LOG_INFO << TAG << f.timesCalled() << " "  << cma.bestSolutionFitness();
            fout << TAG << f.timesCalled() << " "  << cma.bestSolutionFitness() << endl;

        }
        // LOG_INFO << "answer = " << cma.bestSolutionFitness();
        LOG_INFO << TAG << " ends";
        return true;
    }


////////////////////////////////////////////////////////////
// class Problem::solve3 implementation
    bool Problem::solve3(Solver* solver) {
        // // For the comparison
        // return solve_simple(solver);
        
        const std::string TAG = "[SOLVER] ";
        std::vector<Sample*> samples = solver->getSamples();
        if (samples.size() == 0) {
            set_isSolved(true);
            LOG_INFO << TAG << "return immediately -- no initial samples! ";
            return false;
        }

        controller::Knowledge* kn = solver->kn();
        const int    DIM    = samples[0]->params.size();
        const int    MU     = solver->mu();
        const int    LAMBDA = solver->lambda();
        const double SIGMA  = solver->sigma();
        const double clusterMinDist = solver->clusterMinDist();
        LOG_INFO << TAG << "DIM " << DIM;
        LOG_INFO << TAG << "MU/LAMBDA = " << MU << "/" << LAMBDA;
        LOG_INFO << TAG << "SIMGA = " << SIGMA;
        LOG_INFO << TAG << "clusterMinDist = " << clusterMinDist;
        BoumanCluster::g_clusterMinDist = clusterMinDist;

        // 0. Filtering
        samples = this->filterUnmatchedSamples(solver->kn(), samples);
        LOG_INFO << TAG << "# filtered sample = " << samples.size();
        if (samples.size() == 0) {
            set_isSolved(true);
            LOG_INFO << TAG << "return immediately -- invalid problem";
            return false;
        }

        Eigen::VectorXd tasks = solver->sampled_tasks();
        LOG_INFO << TAG << "sampled tasks = " << IO(tasks);
        solver->evaluator()->reevaluate(samples, tasks, false);
        dumpSamples(samples);

        // 1. Initial Clustering for finding models
        // std::vector<ModelCombined*> models
        models = solve3_create_models_cluster(solver, samples);

        // 2. Set the current model
        set_nextStyleRequested(false);
        set_terminateRequested(false);
        currentModelIndex = 0;
        ModelCombined* md = models[currentModelIndex];
        solve3_adapt_model(md, solver);
        md->line->updateModel(kn);
        solver->set_motionUpdatedIteration(0);


        // 2. For the entire loop
        const int MAX_LOOP = 15;
        std::ofstream fout("opt.csv");
        std::ofstream fout_line("opt_lines.csv");
        std::ofstream fout_sample("opt_samples.csv");
        while(md->loop < MAX_LOOP && !terminateRequested()) {
            int loop = md->loop;
            LOG_INFO << TAG << "[Model : " << currentModelIndex << "] "
                     << "iteration " << md->loop << "========================";

            // Sampling
            std::vector<double> volumns = md->bestValueArray();
            double max_value = volumns[0];
            double avg_value = 0.0;
            fout << md->loop;
            for (int i = 0; i < volumns.size(); i++) {
                LOG_INFO << TAG << i << " : " << volumns[i];
                fout << ", " << volumns[i];
                avg_value += volumns[i];
                max_value = std::max(max_value, volumns[i]);
            }
            avg_value /= (double)volumns.size();
            fout << ", " << md->getNumActivatedCovs();
            fout << ", " << avg_value;
            LOG_INFO << TAG << "# Activated Covs = " << md->getNumActivatedCovs();
            LOG_INFO << TAG << "Average Cost = " << avg_value;
            // fout << endl;

            // if (max_value < 0.01) {
            //     LOG_INFO << TAG << "Solver3 achieved a big optimization: max_value = "
            //              << max_value;
            //     break;
            // }


            std::vector<Sample*> population;
            std::vector<int>     population_index;
            for (int i = 0; i < LAMBDA; i++) {
                int ptIndex = randomByVolumns(volumns);
                // LOG_INFO << TAG << i << " : from " << ptIndex;
                ModelPoint* cov = md->covs[ptIndex];
                Sample* s = cov->create();
                population_index.push_back(ptIndex);
                population.push_back(s);
                // LOG_INFO << TAG << "         " << s->toString();
            }

            solver->evaluator()->evaluate( population );
            solver->evaluator()->reevaluate( population, tasks , false);

            for (int i = 0; i < population.size(); i++) {
                Sample* s = population[i];
                LOG_INFO << TAG << i << "/" << LAMBDA << " "
                         << "(" << population_index[i] << ") "
                         << s->toString();
            }

            md->samples.insert(md->samples.end(),
                               population.begin(), population.end());
            samples.insert(samples.end(), population.begin(), population.end());
            // delete md;
            // md = solve3_create_model(solver, samples);

            solve3_update_model_params(md, solver);
            solve3_adapt_model(md, solver);

            // For debug purpose
            {
                std::string TAG = "[VERIFY] ";
                std::vector<Sample*> solutions;
                for (int i = 0; i < tasks.size(); i++) {
                    // Fetch the current optimal parameter
                    double th = tasks(i);
                    Eigen::VectorXd params = (md->line->alpha) + th * (md->line->beta);
                    // Create a sample
                    Sample* s = new Sample(kn->simpack()->con());
                    s->params = params;
                    solutions.push_back(s);
                    LOG_INFO << TAG << i << " " << th << " : " << IO(s->params);
                }
                solver->evaluator()->evaluate( solutions );
                solver->evaluator()->reevaluate( solutions, tasks , false);

                double avg_value = 0.0;
                fout << ", " << "N";
                for (int i = 0; i < solutions.size(); i++) {
                    double th = tasks(i);
                    Sample* s = solutions[i];
                    double value = s->tvalues(i);
                    LOG_INFO << TAG << i << " " << th << " " << value << " : "
                             << IO(s->tvalues) << " :: "
                             << s->toString();
                    fout << ", " << value;
                    avg_value += value;
                }
                avg_value /= (double)solutions.size();
                LOG_INFO << TAG << "average = " << avg_value;
                fout << ", " << avg_value;
                fout << endl;
            }


            md->loop++;

            // Debug
            {
                Eigen::VectorXd p0 = md->line->alpha;
                Eigen::VectorXd p1 = md->line->alpha + md->line->beta;
                fout_line << md->loop;
                for (int i = 0; i < p0.size(); i++) fout_line << ", " << p0(i);
                for (int i = 0; i < p1.size(); i++) fout_line << ", " << p1(i);
                fout_line << endl;

                for (int i = 0; i < population.size(); i++) {
                    Sample* s = population[i];
                    fout_sample << md->loop;
                    for (int j = 0; j < s->params.size(); j++) {
                        fout_sample << ", " << s->params(j);
                    }
                    fout_sample << ", " << s->value;
                    fout_sample << endl;
                }

            }
                


            if (nextStyleRequested()) {
                LOG_INFO << TAG << "nextStyleRequest is detected !!!!";
                // currentModelIndex = (currentModelIndex + 1) % models.size();
                LOG_INFO << TAG << "currentModelIndex = " << currentModelIndex;
                // for (int i = 0; i < models.size(); i++) {
                //     ModelCombined* now = models[i];
                //     LOG_INFO << TAG << i << " a, b = " << IO(now->line->alpha)
                //              << " " << IO(now->line->beta);
                // }
                md = models[currentModelIndex];
                set_nextStyleRequested(false);
                // md->line->updateModel(kn);
                // solver->set_motionUpdatedIteration(md->loop);
                // md->loop++;
            } else {
                if (md->loop % 3 == 0) {
                    md->line->updateModel(kn);
                    solver->set_motionUpdatedIteration(md->loop);
                }
            }


        } // for (int loop = 0;

        md->line->updateModel(kn);
        solver->set_motionUpdatedIteration(MAX_LOOP);
        set_isSolved(true);

        return true;
    }


    int Problem::changeToNextStyle(Solver* solver) {
        set_nextStyleRequested(true);

        controller::Knowledge* kn = solver->kn();
        std::string TAG = "[SOLVER::changeToNextStyle] ";
        LOG_INFO << TAG << " requested";
        if (models.size() == 0) {
            LOG_WARNING << "We do not have models: models.size() = 0";
            return -1;
        }

        currentModelIndex = (currentModelIndex + 1) % models.size();
        ModelCombined* md = models[currentModelIndex];
        
        // For debuging purpose
        for (int i = 0; i < models.size(); i++) {
            ModelCombined* now = models[i];
            LOG_INFO << TAG << i << " a, b = " << IO(now->line->alpha)
                     << " " << IO(now->line->beta);
            md->line->updateModel(kn);
            solver->set_motionUpdatedIteration(md->loop);
        }
        return currentModelIndex;
    }

    int Problem::terminateOpt(Solver* solver) {
        set_terminateRequested(true);
        LOG_INFO << FUNCTION_NAME() << " terminateRequested = 1";
        return 0;
    }

    void Problem::solve3_regression(ModelCombined* md,
                                    Solver* solver,
                                    std::vector<Sample*>& samples) {
        Eigen::VectorXd tasks  = solver->sampled_tasks();
        std::set<Sample*> set_goodsamples;
        for (int i = 0; i < tasks.size(); i++) {
            std::vector<Sample*> goods = selectByTaskValue(samples, i, 1, 1);
            FOREACH(Sample* s, goods) {
                set_goodsamples.insert(s);
            }
        }
        std::vector<Sample*> goodsamples;
        goodsamples.insert(goodsamples.end(),
                           set_goodsamples.begin(), set_goodsamples.end());

        LOG_INFO << "Selected samples for Regression";
        dumpSamples(goodsamples);
        md->line->fit(goodsamples);
    }

    void Problem::solve3_cov(ModelCombined* md,
                             int index,
                             Solver* solver,
                             std::vector<Sample*>& samples) {
        Eigen::VectorXd tasks = solver->sampled_tasks();
        const int       MU    = solver->mu();
        const int       DIM    = samples[0]->params.size();
        const double    SIGMA = solver->sigma();

        double task = tasks(index);
        Eigen::VectorXd pt_on_line = md->line->predict( task );
        std::vector<Sample*> goods = selectByTaskValue(samples, index, 4, 8);
        ModelPoint* cov = md->covs[index];
        cov->init(SIGMA, MU, index, pt_on_line, goods);
        md->activate(index);

        LOG_INFO << "cov = " << cov->C;
        // LOG_INFO << "Selected sample for Covariance matrices";
        // dumpSamples(goods);

        // ModelPoint* cov = new ModelPoint(DIM, solver->kn());
        // cov->T = task;
        // cov->init(SIGMA, MU, index, pt_on_line, goods);
        // md->covs[index] = cov;
    }

    bool criteriaRegError(ModelCombined* lhs, ModelCombined* rhs) {
        return (lhs->line->regressionError < rhs->line->regressionError);
    }
        
    
    std::vector<ModelCombined*>
    Problem::solve3_create_models_cluster(Solver* solver, std::vector<Sample*>& samples) {
        std::string TAG = "[SOLVER::clustering] ";
        
        LOG_INFO << TAG;

        if (solver->clusterMode() == 0) {
            LOG_INFO << "Decided not to do clustering...";
            std::vector<ModelCombined*> ret;
            if (isDegenerated(solver)) {
                std::vector<Sample*> good
                    = selectByTaskRange(samples, 0.0, 0.3, 8, 16);
                LOG_INFO << TAG << " ==== Samples (Degenerate) ====";
                dumpSamples(good);
                LOG_INFO << TAG << " ================";
                ModelCombined* curr = solve3_create_model(solver, good);
                ret.push_back(curr);
            } else {
                std::vector<Sample*> top = selectByTaskRange(samples, 0.0, 0.3, 4, 8);
                LOG_INFO << TAG << " ==== TOP ====";
                dumpSamples(top);
                std::vector<Sample*> mid = selectByTaskRange(samples, 0.3, 0.7, 4, 8);
                LOG_INFO << TAG << " ==== MID ====";
                dumpSamples(mid);

                std::vector<Sample*> bot = selectByTaskRange(samples, 0.7, 1.0, 4, 8);
                LOG_INFO << TAG << " ==== BOT ====";
                dumpSamples(bot);
                std::vector<Sample*> good = add(top, mid, bot);
                LOG_INFO << TAG << " ==== Samples (All)  ====";
                dumpSamples(good);
                LOG_INFO << TAG << " ================";
                ModelCombined* curr = solve3_create_model(solver, good);
                ret.push_back(curr);
            }
            return ret;
        }


        if (isDegenerated(solver)) {
            std::vector<ModelCombined*> ret;
            LOG_INFO << TAG << " Degenerate Case !!!!!!!!!!!!!!!!!";
            std::vector<Sample*> good
                = selectByTaskRange(samples, 0.0, 0.3, 8, 16);
            LOG_INFO << TAG << " ==== Good ====";
            dumpSamples(good);
            LOG_INFO << TAG << " ==============";

            BoumanCluster bc_good(good);
            bc_good.cluster();
            for (int i = 0; i < bc_good.numClasses(); i++) {
                std::vector<Sample*> clustered = bc_good.getCluster(i);
                LOG_INFO << TAG << " ==== Case " << i << " ====";
                dumpSamples(clustered);
                LOG_INFO << TAG << " ================";
                ModelCombined* curr = solve3_create_model(solver, clustered);
                ret.push_back(curr);
            }
            // exit(0);
            return ret;
        }
        
        // 1. Grouping into three categories
        std::vector<Sample*> top = selectByTaskRange(samples, 0.0, 0.3, 4, 8);
        LOG_INFO << TAG << " ==== TOP ====";
        BoumanCluster bc_top(top);
        bc_top.cluster();

        dumpSamples(top);
        std::vector<Sample*> mid = selectByTaskRange(samples, 0.3, 0.7, 4, 8);
        LOG_INFO << TAG << " ==== MID ====";
        dumpSamples(mid);

        std::vector<Sample*> bot = selectByTaskRange(samples, 0.7, 1.0, 4, 8);
        LOG_INFO << TAG << " ==== BOT ====";
        dumpSamples(bot);
        BoumanCluster bc_bot(bot);
        bc_bot.cluster();

        std::vector<ModelCombined*> ret;
        for (int i = 0; i < bc_top.numClasses(); i++) {
            for (int j = 0; j < bc_bot.numClasses(); j++) {
                std::vector<Sample*> t = bc_top.getCluster(i);
                std::vector<Sample*> b = bc_bot.getCluster(j);
                std::vector<Sample*> clustered = add(t, mid, b);
                LOG_INFO << TAG << " ==== Case " << i << " " << j << " ====";
                dumpSamples(clustered);
                ModelCombined* curr = solve3_create_model(solver, clustered);
                ret.push_back(curr);
            }
        }

        // sort(ret.begin(), ret.end(), criteriaRegError);
        if (ret.size() == 4) {
            LOG_INFO << TAG << "We have four regressions: cut down to two";
            for (int i = 0; i < ret.size(); i++) {
                ModelCombined* md = ret[i];
                LOG_INFO << TAG << "Model " << i << " = " << md->line->regressionError;
            }
            double v0 = ret[0]->line->regressionError;
            double v1 = ret[1]->line->regressionError;
            double v2 = ret[2]->line->regressionError;
            double v3 = ret[3]->line->regressionError;

            std::vector<ModelCombined*> ret2;
            if (v0 + v3 < v1 + v2) {
                LOG_INFO << TAG << "Select 0 and 3";
                ret2.push_back(ret[0]);
                ret2.push_back(ret[3]);
            } else {
                LOG_INFO << TAG << "Select 1 and 2";
                ret2.push_back(ret[1]);
                ret2.push_back(ret[2]);
            }
            ret = ret2;
        }

        return ret;
    }

    ModelCombined*
    Problem::solve3_create_model(Solver* solver,
                                 std::vector<Sample*>& samples) {
        const std::string TAG = "[SOLVER::create] ";
        // Fetch some variables
        controller::Knowledge* kn     = solver->kn();
        Eigen::VectorXd        tasks  = solver->sampled_tasks();
        const int              DIM    = samples[0]->params.size();
        const int              MU     = solver->mu();
        const int              LAMBDA = solver->lambda();
        const double           SIGMA  = solver->sigma();

        // Initialize the model
        ModelCombined* md = new ModelCombined(DIM, tasks, kn);
        md->samples = samples;

        // 1.1. Initial selection and regression
        solve3_regression(md, solver, md->samples);

        // 1.2.a. If it is degenerate, return the only one covariance matrix
        if (isDegenerated(solver)) {
            md->isDegenerated = true;
            int i = 0;
            LOG_INFO << TAG << "Cov " << i << " created (Degenerate)";
            solve3_cov(md, i, solver, md->samples);
            return md;
        }

        // 1.2. Estimate the covariances for activated points
        for (int i = 0; i < tasks.size(); i++) {
            if (i == 0 || i == (tasks.size() - 1)) {
                LOG_INFO << TAG << "Cov " << i << " created";
                solve3_cov(md, i, solver, md->samples);
            }
        }

        // 1.3. Interpolate the covariances for not-activated points
        for (int i = 0; i < tasks.size(); i++) {
            if (md->isActivated(i)) {
                continue;
            }
            int l = md->left(i);
            int r = md->right(i);
            double w = (tasks(i) - tasks(l)) / (tasks(r) - tasks(l)); 
            LOG_INFO << TAG << "Task " << i << " interpolated "
                     << "l/r = " << l << "/" << r;
            delete md->covs[i];
            ModelPoint* lhs = md->covs[l];
            ModelPoint* rhs = md->covs[r];
            md->covs[i] = ModelPoint::interpolate(lhs, rhs, w);

            std::vector<Sample*> goods = selectByTaskValue(samples, i, 4, 8);
            md->covs[i]->updateBestValues(goods, i);
        }

        return md;
    }

    void
    Problem::solve3_update_model_params(ModelCombined* md,
                                        Solver* solver) {
        const std::string TAG = "[SOLVER::update] ";
        // Fetch some variables
        controller::Knowledge* kn     = solver->kn();
        Eigen::VectorXd        tasks  = solver->sampled_tasks();
        const int              DIM    = md->DIM;
        const int              MU     = solver->mu();
        const int              LAMBDA = solver->lambda();
        const double           SIGMA  = solver->sigma();

        // 1.1. Initial selection and regression
        solve3_regression(md, solver, md->samples);
        LOG_INFO << TAG << "regressionError = " << md->line->regressionError;

        // 1.2.a. If it is degenerate, return the only one covariance matrix
        if (isDegenerated(solver)) {
            int i = 0;
            LOG_INFO << TAG << "Cov " << i << " created";
            solve3_cov(md, i, solver, md->samples);
            return;
        }

        // 1.2. Estimate the covariances for activated points
        for (int i = 0; i < tasks.size(); i++) {
            if (i == 0 || i == (tasks.size() - 1)) {
                LOG_INFO << TAG << "Cov " << i << " created";
                solve3_cov(md, i, solver, md->samples);
            }
        }

        // 1.3. Interpolate the covariances for not-activated points
        for (int i = 0; i < tasks.size(); i++) {
            if (md->isActivated(i)) {
                continue;
            }
            int l = md->left(i);
            int r = md->right(i);
            double w = (tasks(i) - tasks(l)) / (tasks(r) - tasks(l)); 
            LOG_INFO << TAG << "Task " << i << " interpolated "
                     << "l/r = " << l << "/" << r;
            delete md->covs[i];
            ModelPoint* lhs = md->covs[l];
            ModelPoint* rhs = md->covs[r];
            md->covs[i] = ModelPoint::interpolate(lhs, rhs, w);

            std::vector<Sample*> goods = selectByTaskValue(md->samples, i, 4, 8);
            md->covs[i]->updateBestValues(goods, i);
        }
    }

    bool Problem::solve3_adapt_model(ModelCombined* md,
                                     Solver* solver) {
        if (isDegenerated(solver)) {
            return false;
        }

        const std::string TAG = "[SOLVER::adapt] ";
        // Fetch some variables
        std::vector<Sample*> samples  = md->samples;
        controller::Knowledge* kn     = solver->kn();
        Eigen::VectorXd        tasks  = solver->sampled_tasks();
        LOG_INFO << TAG << "# samples = " << samples.size();

        // Testing
        std::vector<double> array_ll;
        for (int i = 0; i < tasks.size(); i++) {
            double task = tasks(i);
            std::vector<Sample*> goods = selectByTaskValue(samples, i, 4, 8);
            LOG_INFO << TAG << "task = " << task;

            ModelPoint* cov = md->covs[i];

            double sum = 0.0;
            double cnt = 0.0;
            FOREACH(Sample* g, goods) {
                // double ll_lhs = lhs ->loglikelihood(g->params);
                // double ll_rhs = rhs ->loglikelihood(g->params);
                // double ll = std::max(ll_lhs, ll_rhs);
                double ll = cov->loglikelihood(g->params);
                sum += ll;
                cnt += 1.0;
                LOG_INFO << TAG << "loglikelihood = "
                         << ll << " <- " << IO(g->params);
            }
            double avg_ll = sum / cnt;
            array_ll.push_back(avg_ll);
            LOG_INFO << TAG << "COV.average = " << avg_ll;
        }

        // Get min/max
        double min_ll     = array_ll[0];
        int    min_index  = 0;
        double min_ll_act = array_ll[0]; // activated[0] == ACTIVATED, always.
        for (int i = 1; i < array_ll.size(); i++) {
            double ll = array_ll[i];
            if (min_ll > ll) {
                min_ll = ll;
                min_index = i;
            }
            if (md->isActivated(i) && min_ll_act > ll) {
                min_ll_act = ll;
            }
        }
        LOG_INFO << "Minimum activated loglikelihood = " << min_ll_act;

        if (min_ll < 0.5 * min_ll_act && md->isActivated(min_index) == false) {
            int index = min_index;
            LOG_INFO << TAG << " !!!! ACTIVATE " << index;
            LOG_INFO << TAG << "rebuilding Covariance...";
            solve3_cov(md, index, solver, samples);
            ModelPoint* cov = md->covs[index];
            LOG_INFO << TAG << "task = " << tasks[index] << endl
                     << IO(cov->m) << endl
                     << cov->C;
            LOG_INFO << TAG << endl;
            return true;
        } else {
            return false;
        }


    }

// class Problem::solve3 ends
////////////////////////////////////////////////////////////

} // namespace solver
