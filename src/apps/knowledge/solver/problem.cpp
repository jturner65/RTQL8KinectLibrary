/* Rtql8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "problem.h"

#include <algorithm>
#include <set>
#include <boost/scoped_ptr.hpp>

#include "utils/UtilsMath.h"

#include "common/app_cppcommon.h"

#include "controller/app_composite_controller.h"
#include "controller/knowledge.h"
#include "controller/simpack.h"
#include "controller/cost.h"

#include "solver.h"
#include "evaluator.h"
#include "explorer.h"
#include "model.h"

namespace solver {

    bool sampleCriteria(Sample* lhs, Sample* rhs) {
        return (lhs->value < rhs->value);
    }
    
////////////////////////////////////////////////////////////
// class Problem implementation
    Problem::Problem()
        : MEMBER_INIT(isSolved, false)
        , MEMBER_INIT(terminateRequested, false)
        , currentModelIndex(0)
    {
    }
    
    Problem::~Problem() {
    }

    bool Problem::solve(Solver* solver) {
        return solve3(solver);
        
        // boost::scoped_ptr<ModelBezier> m(new ModelBezier());
        // // 0. Filtering
        // std::vector<Sample*> samples = solver->getSamples();
        // samples = this->filterUnmatchedSamples(solver->kn(), samples);
        // LOG_INFO << "# filtered sample = " << samples.size();
        // if (samples.size() == 0) {
        //     set_isSolved(true);
        //     LOG_INFO << "return immediately -- invalid problem";
        //     return false;
        // }
        // controller::AppCompositeController* cc = solver->kn()->simpack()->con();

        // samples = filterInvalidSamples(samples);
        // solver->evaluator()->reevaluate(samples);
        // samples = filterBadSamples(samples);
        // dumpSamples(samples);
        // // exit(0);
        
        // // 1. Formulation
        // int dim = samples[0]->params.size();
        // LOG_INFO << "formulating the problem... dim = " << dim;
        // m->initialize(dim);

        // // 2. Fitting
        // LOG_INFO << "fitting the problem...";
        // m->fit(samples);

        // // 3. Updating the original controller
        // LOG_INFO << "updating the knowledge...";
        // m->update(solver->kn());

        // // 4. Mark as solved
        // set_isSolved(true);
        // LOG_INFO << "Optimization Problem Solved!!!";

        // solver->kn()->pushHint("prefixclear");
        
        return true;
    }

    bool Problem::solve2(Solver* solver) {
        const std::string TAG = "[SOLVER] ";
        boost::scoped_ptr<ModelBezier> m(new ModelBezier());

        // 0. Filtering
        std::vector<Sample*> samples = solver->getSamples();
        samples = this->filterUnmatchedSamples(solver->kn(), samples);
        LOG_INFO << TAG << "# filtered sample = " << samples.size();
        if (samples.size() == 0) {
            set_isSolved(true);
            LOG_INFO << TAG << "return immediately -- invalid problem";
            return false;
        }

        // 0.1. reevaluate the filtered samples
        solver->evaluator()->reevaluate(samples);
        // // 0.2. after filtering, dump samples for debugging
        // dumpSamples(samples);

        // 1. Initializaing model
        // Lambda > = 2, population size, sample size, # of offspring.
        // Mu <= Lambda, parent number, number of selected point in population.
        const int    DIM    = samples[0]->params.size();
        const int    MU     = solver->mu();
        const int    LAMBDA = solver->lambda();
        const double SIGMA  = solver->sigma();
        LOG_INFO << TAG << "DIM " << DIM;
        LOG_INFO << TAG << "MU/LAMBDA = " << MU << "/" << LAMBDA;
        LOG_INFO << TAG << "SIMGA = " << SIGMA;
        
        Eigen::VectorXd tasks = solver->sampled_tasks();
        LOG_INFO << TAG << "sampled tasks = " << IO(tasks);
        std::vector<ModelPoint*> pts;
        for (int i = 0; i < tasks.size(); i++) {
            double task = tasks(i);
            LOG_INFO << TAG << "task = " << task;

            ModelPoint* pt = new ModelPoint(DIM, solver->kn());
            pt->T = task;
            std::vector<Sample*> current = selectByClosestTask(samples, tasks, i);
            pt->init(SIGMA, MU, current);
            LOG_INFO << "Initializing cma OK";
            dumpSamples(current);
            cout << "dumpSamples OK" << endl;
            if (current.size() > 0) {
                pts.push_back(pt);
            } else {
                LOG_WARNING << "We will ignore this point due to the lack of samples";
                continue;
            }
        }

        LOG_INFO << TAG << "start the loop....";

        // 2. Loop
        const int MAX_LOOP = 5;
        for (int loop = 0; loop < MAX_LOOP; loop++) {
            LOG_INFO << TAG << "==== iteration " << loop << " ====";
            // Collect the volumns
            std::vector<double> volumns;
            for (int i = 0; i < pts.size(); i++) {
                ModelPoint& pt = *(pts[i]);
                // double vol = pt.volumn();
                double vol = pt.bestFitness;
                volumns.push_back(vol);
            }
            
            std::vector<Sample*> population;
            for (int i = 0; i < LAMBDA; i++) {
                int ptIndex = randomByVolumns(volumns);
                ModelPoint& pt = *(pts[ptIndex]);
                Sample* s = pt.create();
                population.push_back(s);
                // LOG_INFO << TAG << i << " : from " << ptIndex;
                // LOG_INFO << TAG << "         " << s->toString();
            }

            solver->evaluator()->evaluate( population );
            solver->evaluator()->reevaluate( population, tasks , false);
            // for (int i = 0; i < population.size(); i++) {
            //     Sample* s = population[i];
            //     LOG_INFO << TAG << i << "/" << LAMBDA << " " << s->toString();
            // }

            for (int i = 0; i < pts.size(); i++) {
                ModelPoint& pt = *(pts[i]);
                double task = tasks(i);
                std::vector<Sample*> current
                    = selectByClosestTask(population, tasks, i);
                std::sort(current.begin(), current.end(), sampleCriteria);


                int lambda = current.size();
                int mu = (lambda + 1) / 2;
                std::vector<Sample*> child = current;
                child.erase(child.begin() + mu, child.end());

                LOG_INFO << TAG << "task = " << task
                         << " mu = " << mu;
                for (int j = 0; j < current.size(); j++) {
                    Sample* s = current[j];
                    LOG_INFO << TAG << j << "/" << LAMBDA << " " << s->toString();
                }

                pt.update(child);
            }
        }

        if (pts.size() == 2) {
            // Regression by hand
            Eigen::VectorXd p0 = pts[0]->bestParams;
            Eigen::VectorXd p1 = pts[1]->bestParams;
            Eigen::VectorXd alpha = p0;
            Eigen::VectorXd beta  = p1 - p0;
            solver->kn()->simpack()->con()->setRegressionParams( alpha, beta );
            LOG_INFO << "p0 = " << IO(p0);
            LOG_INFO << "p1 = " << IO(p1);
            LOG_INFO << "alpha = " << IO(alpha);
            LOG_INFO << "beta  = " << IO(beta);
        } else {
            Eigen::VectorXd p0 = pts[0]->bestParams;
            Eigen::VectorXd alpha = p0;
            Eigen::VectorXd beta  = Eigen::VectorXd::Zero( DIM );
            solver->kn()->simpack()->con()->setRegressionParams( alpha, beta );
            LOG_INFO << "p0 = " << IO(p0);
            LOG_INFO << "alpha = " << IO(alpha);
            LOG_INFO << "beta  = " << IO(beta);
        }
        // Finale
        LOG_INFO << TAG << "finished :)";
        // exit(0);
        return true;
    }


    std::vector<Sample*>
    Problem::filterUnmatchedSamples(controller::Knowledge* kn,
                                    const std::vector<Sample*>& samples) { 
        std::vector<Sample*> filtered;
        const std::vector<std::string>& lhs = kn->simpack()->con()->getDimNames();
        for (int i = 0; i < lhs.size(); i++) {
            cout << i << " : " << lhs[i] << endl;
        }

        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            const std::vector<std::string>& rhs = s->dimensions;
            if (lhs.size() != rhs.size()) {
                continue;
            }

            bool isGood = true;
            for (int j = 0; j < lhs.size(); j++) {
                if (lhs[j] != rhs[j]) {
                    isGood = false;
                    break;
                }
            }
            if (isGood == false) {
                continue;
            }

            // If it is matched
            filtered.push_back(s);
        }
        return filtered;
    }

    std::vector<Sample*>
    Problem::filterInvalidSamples(const std::vector<Sample*>& samples) {
        std::vector<Sample*> filtered;

        const double FILTER_CRITERIA = 10.0;
        FOREACH(Sample* s, samples) {
            if (s->value < FILTER_CRITERIA) {
                filtered.push_back(s);
            }
        }
        
        return filtered;
        
    }

    std::vector<Sample*>
    Problem::filterBadSamples(const std::vector<Sample*>& samples) {
        std::vector<Sample*> filtered;
        double min_value = 99.0;
        FOREACH(Sample* s, samples) {
            min_value = std::min(min_value, s->value);
        }

        const double FILTER_RATIO = 5.0;
        // const double MIN_CRITERIA = 0.01;
        const double MIN_CRITERIA = 0.1;
        const double FILTER_CRITERIA = std::max(min_value * FILTER_RATIO, MIN_CRITERIA);

        LOG_INFO << "min_value = " << min_value << " CRITERIA = " << FILTER_CRITERIA;
        FOREACH(Sample* s, samples) {
            if (s->value < FILTER_CRITERIA) {
                filtered.push_back(s);
            }
        }
        
        return filtered;
    }

    std::vector<Sample*>
    Problem::selectByClosestTask(const std::vector<Sample*>& samples,
                                 const Eigen::VectorXd& tasks,
                                 int targetIndex) {
        std::vector<Sample*> ret;
        
        FOREACH(Sample* s, samples) {
            double min_dist = 10000.0;
            int index = -1;
            for (int i = 0; i < tasks.size(); i++) {
                double d = fabs(tasks(i) - s->task);
                if (d < min_dist) {
                    min_dist = d;
                    index = i;
                }
            }
            if (index == targetIndex) {
                ret.push_back(s);
            }
        }
        return ret;
    }

    int g_taskValueComp_index = -1;
    bool taskValueComp(const Sample* lhs, const Sample* rhs) {
        int idx = g_taskValueComp_index;
        return (lhs->tvalues[idx] < rhs->tvalues[idx]);
    }

    std::vector<Sample*>
    Problem::selectByTaskValue(std::vector<Sample*>& samples,
                               int idx,
                               int min_num, int max_num) {
        g_taskValueComp_index = idx;
        sort( samples.begin(), samples.end(), taskValueComp );

        if (samples.size() == 0) {
            return std::vector<Sample*>();
        }

        const double HARD_BOUND = 0.1;
        const double REL_BOUND  = samples[0]->tvalues[idx] * 1.5;
        const int    MIN_NUM    = std::min((int)min_num, (int)samples.size());
        const int    MAX_NUM    = max_num;

        std::vector<Sample*> ret;
        for (int i = 0; i < MIN_NUM; i++) {
            ret.push_back(samples[i]);
        }
        for (int i = MIN_NUM; i < MAX_NUM; i++) {
            if (samples.size() <= i) break;
            double v= samples[i]->tvalues[idx];
            if (v > HARD_BOUND || v > REL_BOUND) {
                break;
            }
            ret.push_back(samples[i]);
        }
        return ret;

    }
    std::vector<Sample*> Problem::selectByTaskRange(std::vector<Sample*>& samples,
                                                    double lo, double hi,
                                                    int min_num, int max_num) {
        double EPS = 0.001;
        std::vector<Sample*> filtered;
        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            if (lo - EPS < s->task && s->task < hi + EPS) {
                filtered.push_back(s);
            }
        }
        if (filtered.size() == 0) {
            LOG_FATAL << FUNCTION_NAME() << " Invalid selection with tasks!!!!!!!";
            return std::vector<Sample*>();
        }

        sort(filtered.begin(), filtered.end(), sampleCriteria);

        const double HARD_BOUND = 0.2;
        const double REL_BOUND  = filtered[0]->value * 3.0;
        const int    MIN_NUM    = std::min((int)min_num, (int)filtered.size());
        const int    MAX_NUM    = max_num;


        std::vector<Sample*> ret;
        for (int i = 0; i < MIN_NUM; i++) {
            ret.push_back(filtered[i]);
        }
        for (int i = MIN_NUM; i < MAX_NUM; i++) {
            if (filtered.size() <= i) break;
            double v= filtered[i]->value;
            if (v > HARD_BOUND || v > REL_BOUND) {
                break;
            }
            ret.push_back(filtered[i]);
        }
        return ret;
    }

    std::vector<Sample*> Problem::add(std::vector<Sample*>& s0,
                                      std::vector<Sample*>& s1,
                                      std::vector<Sample*>& s2) {
        std::vector<Sample*> ret;
        ret.insert(ret.end(), s0.begin(), s0.end());
        ret.insert(ret.end(), s1.begin(), s1.end());
        ret.insert(ret.end(), s2.begin(), s2.end());
        return ret;
    }

    void Problem::dumpSamples(const std::vector<Sample*>& samples) {
        LOG_INFO << "---- dumpSamples() ----";
        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            LOG_INFO << i << " : "
                     << s->task << " "
                     << IO(s->tvalues) << " <- "
                     << IO(s->params);
                     // << s->toString();
        }
        LOG_INFO << "-----------------------" << endl;
    }

    void Problem::dumpSamples(const std::set<Sample*>& samples) {
        std::vector<Sample*> temp;
        FOREACH(Sample* s, samples) {
            temp.push_back(s);
        }
        dumpSamples(temp);
    }

    int Problem::randomByVolumns(const std::vector<double>& volumns) {
        double sum = 0.0;
        for (int i = 0; i < volumns.size(); i++) {
            sum += volumns[i];
        }
        std::vector<double> normalized;
        double sum2 = 0.0;
        for (int i = 0; i < volumns.size(); i++) {
            sum2 += volumns[i];
            normalized.push_back(sum2 / sum);
            // cout << i << " : " << normalized[i] << endl;
        }

        double r = rtql8::utils::random(0.0, 1.0);
        // cout << "r = " << r << endl;

        for (int i = 0; i < normalized.size(); i++) {
            double n = normalized[i];
            if (r < n) {
                return i;
            }
        }
        
        return volumns.size() - 1;
    }

    bool Problem::isRanged(Solver* solver) {
        controller::Knowledge* kn = solver->kn();
        controller::SimPack* sp = kn->simpack();
        return sp->con()->cost()->isRanged();
        // return true;
    }

    bool Problem::isDegenerated(Solver* solver) {
        return !isRanged(solver);
    }
    
// class Problem ends
////////////////////////////////////////////////////////////
    

    
} // namespace solver





