/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "explorer.h"

#include <algorithm>
// #include <boost/function.hpp>
// #include <boost/lambda/lambda.hpp>
#include <boost/algorithm/string.hpp>

#include "common/app_cppcommon.h"
#include "controller/app_composite_controller.h"
#include "controller/knowledge.h"
#include "controller/simpack.h"

#include "sample.h"
#include "solver.h"
#include "evaluator.h"

namespace solver {
    
////////////////////////////////////////////////////////////
// class Explorer implementation
    Explorer::Explorer(Solver* _solver)
        : MEMBER_INIT_ARG(solver)
    {
        // set_mykn( kn()->duplicate() );
        // LOG_INFO << FUNCTION_NAME() << " OK";
        // updateMyKnowledge();

        // set_remotedb( new RemoteDB() );
        // if (remotedb()) {
        //     bool result = remotedb()->connect();
        //     remotedb()->registerKnowledge(mykn());
        //     remotedb()->deleteAllSamples();
        // }
        LOG_INFO << FUNCTION_NAME() << " OK";
    }
    
    Explorer::~Explorer() {
    }


    void Explorer::filterRigNamesDupicated(std::vector<std::string>& names) {
        std::vector<std::string> ret;

        std::vector<std::string> dims = kn()->simpack()->con()->getDimNames();
        for (int i = 0; i < names.size(); i++) {
            const std::string& name = names[i];
            bool alreadyIncluded = false;
            for (int j = 0; j < dims.size(); j++) {
                const std::string& dim = dims[j];
                if (name == dim) {
                    alreadyIncluded = true;
                    break;
                }
            }
            if (alreadyIncluded == false) {
                ret.push_back(name);
            }
        }
        
        if (ret.size() > 0) {
            names = ret;
        }
    }

    void Explorer::filterRigNamesByPrefix(std::vector<std::string>& names) {
        std::vector<std::string> ret;
        for (int i = 0; i < names.size(); i++) {
            if (boost::starts_with(names[i], rig_prefix)) {
                ret.push_back(names[i]);
            }
        }
        if (ret.size() > 0) {
            names = ret;
        }
    }

    bool Explorer::mainloop() {
        // Parameters
        srand( (unsigned int) time (NULL) );
        const int DIM = kn()->simpack()->con()->dim();
        const int N_ITER_SAMPLES = 16;
        int N_CURRENT = (DIM == 0) ? 0 : 4;
        if (rig_prefix.length() > 0) {
            N_CURRENT = 0;
        }
        if (rig_prefix == "NO_NEW_RIGS") {
            N_CURRENT = N_ITER_SAMPLES;
        }

        std::vector<Sample*> curr_samples;
        std::vector<std::string> rig_names;
        rig_names = kn()->getPrimitiveControllerNames();

        filterRigNamesDupicated(rig_names);
        filterRigNamesByPrefix(rig_names);

        // Generate samples
        const int NUMPHASES = kn()->simpack()->con()->numPhases();
        for (int i = 0; i < N_CURRENT; i++) {
            Sample* e = new Sample( kn()->simpack()->con() );
            e->generateRandomParams();
            // e->params = Eigen::VectorXd::Random(e->numDims());
            // Eigen::VectorXd lower
            // for (int j = 0; j < e->params.size(); j++) {
            //     double p = e->params(j);
            //     double w = (p + 1.0) / (2.0);
            //     double q = lower(i) + w * (upper(i) - lower(i));
            //     e->params(j) = q;
            // }
            // LOG_INFO << "params = " << IO(e->params)
            //          << " (" << IO(e->lower) << "/" << IO(e->upper) << ")";
            curr_samples.push_back( e  );
        }

        const int m = rig_names.size();
        if (m == 0) {
            LOG_FATAL << "no available rigs!! m= 0";
            exit(0);
        }

        for (int i = N_CURRENT; i < N_ITER_SAMPLES; i++) {
            int j = rand() % m;
            Sample* e = new Sample( kn()->simpack()->con(), rig_names[j] );
            e->generateRandomParams();
            // for (int j = 0; j < NUMPHASES; j++) {
            //     if (e->params(j) < 0) {
            //         e->params(j) = fabs(e->params(j));
            //     }
            // }
            // LOG_INFO << "params = " << IO(e->params)
            //          << " (" << IO(e->lower) << "/" << IO(e->upper) << ")";
            curr_samples.push_back( e  );
        }

        // // Print it out! for debug..
        // for (int i = 0; i < curr_samples.size(); i++) {
        //     const Sample* s = curr_samples[i];
        //     LOG_INFO << i << " : " << s->toString();
        // }

        // Evaluate samples
        solver()->evaluator()->evaluate( curr_samples );
        // // Evaluate samples
        // for (int i = 0; i < curr_samples.size(); i++) {
        //     Sample& s = curr_samples[i];
        //     LOG_INFO << "evaluating " << i;
        //     // s.evaluate(mykn());
        //     s.evaluateDB(mykn(), remotedb());
        //     LOG_INFO << "evaluating done " << i;
        // }

        // Print it out! for debug..
        for (int i = 0; i < curr_samples.size(); i++) {
            const Sample* s = curr_samples[i];
            LOG_INFO << i << " : " << s->toString();
        }

        // Put it back to the solver/database
        for (int i = 0; i < curr_samples.size(); i++) {
            Sample* s = curr_samples[i];
            // samples.push_back(s);
            solver()->addSample(s);
        }

        // // Generate exploration samples
        // BOOST_FOREACH(const std::string& name, rig_names) {
        //     cout << name << " ";
        // }
        // cout << endl;
        dropInvalidSamples();
        while(1) {
            std::string hint = kn()->fetchHint();
            if (hint == "") {
                break;
            }
            LOG_INFO << ")))) EXPLORER: hint is detected: " << hint;
            std::vector<std::string> tokens;
            boost::split(tokens, hint, boost::is_any_of("\n\t :"));
            std::string cmd = tokens[0];
            if (cmd == "prefix") {
                this->rig_prefix = tokens[1];
                LOG_INFO << ")))) EXPLORER: hint prefix = [" << rig_prefix << "]";
            } else if (cmd == "prefixclear") {
                this->rig_prefix = "";
                LOG_INFO << ")))) EXPLORER: hint prefixclear";
            } else if (cmd == "clear") {
                dropAllSamples();
                LOG_INFO << ")))) EXPLORER: hint clear";
            }

        }
        LOG_INFO << FUNCTION_NAME() << " : # explored = " << solver()->numSamples();
        return true;
    }

    void Explorer::dropAllSamples() {
        std::vector<Sample*>& samples = solver()->getSamples();
        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            delete s;
        }
        samples.clear();
        solver()->replaceSamples(samples);
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Explorer::dropInvalidSamples() {
        const std::vector<std::string>& lhs = kn()->simpack()->con()->getDimNames();
        for (int i = 0; i < lhs.size(); i++) {
            LOG_INFO << "dropInvalidSamples: rig " << i << " = " << lhs[i];
        }

        int origNum = solver()->numSamples();
        std::vector<Sample*>& samples = solver()->getSamples();
        std::vector<Sample*> goodsamples;

        for (int i = 0; i < samples.size(); i++) {
            Sample* s = samples[i];
            const std::vector<std::string>& rhs = s->dimensions;
            // Must check w.r.t. left hand side
            // The end of rhs could be the future prediction
            bool needToRemoved = false;
            for (int j = 0; j < lhs.size(); j++) {
                if (j >= rhs.size()) {
                    // cout << "delete " << s->toString() << endl;
                    needToRemoved = true;
                    break;
                }
                if (lhs[j] != rhs[j]) {
                    // cout << "delete " << s->toString() << endl;
                    needToRemoved = true;
                    break;
                }
            }
            if (needToRemoved == false) {
                goodsamples.push_back(s);
            }
        }
        solver()->replaceSamples(goodsamples);
        // samples = goodsamples;
        // using namespace boost::lambda;
        // std::vector<Sample*>::iterator new_end
        //     = std::remove(samples.begin(), samples.end(), _1 == NULL);

        // samples.erase(new_end, samples.end());
        LOG_INFO << "After drop invalid samples: "
                 << origNum << " -> " << solver()->numSamples();
    }

// Helper functions
    controller::Knowledge* Explorer::kn() {
        return solver()->kn();
    }

// class Explorer ends
////////////////////////////////////////////////////////////
    
    
} // namespace solver



