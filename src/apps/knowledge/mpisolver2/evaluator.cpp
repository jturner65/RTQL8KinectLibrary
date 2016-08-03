#include "evaluator.h"

#include <algorithm>

#include "common/app_cppcommon.h"
#include "controller/SimPack.h"
#include "controller/controller"

#include "commandcenter.h"
#include "database.h"

namespace mpisolver2 {
    Evaluator::Evaluator(CommandCenter* _cmd)
        : MEMBER_INIT(cmd, _cmd)
    {
        param_names.clear();
        cmd()->simpack()->con()->phase()->get_param_names( &param_names );
    }

    Evaluator::~Evaluator() {
    }

    bool Evaluator::expandParams() {
        std::vector<std::string>& lhs = param_names;
        std::vector<std::string> rhs;
        cmd()->simpack()->con()->phase()->get_param_names( &rhs );

        if (lhs.size() == rhs.size()) {
            return false;
        }

        LOG(INFO) << FUNCTION_NAME();
        // Build index map
        std::vector<int> indexmap;
        FOREACH(const std::string& name, rhs) {
            std::vector<std::string>::iterator i = std::find(lhs.begin(), lhs.end(), name);
            int index = -1;
            if (i != lhs.end()) {
                index = i - lhs.begin();
            }
            
            cout << "name " << name << " in original coordinate = " << index << endl;
            indexmap.push_back(index);
        }
        const int dim = cmd()->dim();

        CHECK_EQ(dim, indexmap.size());

        for (int i = 0; i < db()->numSamples(); i++) {
            Sample* s = db()->sample(i);

            Eigen::VectorXd newparams = Eigen::VectorXd::Zero(dim);
            std::vector<bool> newIsParamUsed;
            for (int j = 0; j < dim; j++) { newIsParamUsed.push_back(false); }

            for (int j = 0; j < dim; j++) {
                int index = indexmap[j];
                if (index == -1) {
                    continue;
                }
                newparams(j) = s->params(index);
                newIsParamUsed[j] = s->isParamUsed[index];
            }

            cout << "Sample " << i << " " << IO(s->params) << " --> " << IO(newparams);
            s->params = newparams;
            s->isParamUsed = newIsParamUsed;
            cout << " " << IO(s->isParamUsedAsEigen()) << endl;
        }
        lhs = rhs;
        LOG(INFO) << FUNCTION_NAME() << " OK";
        return true;
    }

    void Evaluator::reevaluate() {
        LOG(INFO) << FUNCTION_NAME();
        expandParams();
        
        const int dim = cmd()->dim();
        
        for (int i = 0; i < db()->numSamples(); i++) {
            Sample* s = db()->sample(i);
            CHECK_EQ( dim, s->params.size() );
            // simpack()->sim()->setSimState(s->finalstate);
            double newvalue = simpack()->con()->phase()->evaluatePhaseStateMap(&(s->finalstate));
//            double newvalue = simpack()->con()->evaluate();
            cout << boost::format("%d: %lf --> %lf") % i % s->value % newvalue << endl;

            s->prevValue = s->value;
            s->value = newvalue;
        }
        LOG(INFO) << FUNCTION_NAME() << " OK";

    }

    int Evaluator::nChildProc() {
        return cmd()->nChildProc();
    }

    controller::SimPack* Evaluator::simpack() {
        return cmd()->simpack();
    }
    
    Database* Evaluator::db() {
        return cmd()->database();
    }

} // namespace mpisolver2
