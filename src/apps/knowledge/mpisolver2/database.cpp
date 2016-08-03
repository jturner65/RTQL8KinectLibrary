#include "database.h"

#include <algorithm>
#include "common/app_cppcommon.h"

namespace mpisolver2 {
    bool betterValue(const Sample* const lhs, const Sample* const rhs) {
        return (lhs->value < rhs->value);
    }

    bool betterPrevValue(const Sample* const lhs, const Sample* const rhs) {
        return (lhs->prevValue < rhs->prevValue);
    }

    Database::Database() {
        clear();
    }

    Database::~Database() {
    }

    void Database::clear() {
        samples.clear();
        reportedTags.clear();
    }

    void Database::add(Sample* const s) {
        samples.push_back(s);
    }

    bool Database::erase(int index) {
    	if (index < 0 || index >= numSamples()) {
    		return false;
    	}
    	samples.erase(samples.begin() + index);
    	return true;
    }

    Sample* Database::bestSample() {
        if (numSamples() == 0) {
            return NULL;
        }
        
        Sample* ret = sample(0);
        for (int i = 1; i < numSamples(); i++) {
            Sample* s = sample(i);
            if (ret->value > s->value) {
                ret = s;
            }
        }
        return ret;
    }

    Sample* Database::bestSampleByPrev() {
        if (numSamples() == 0) {
            return NULL;
        }
        
        Sample* ret = sample(0);
        for (int i = 1; i < numSamples(); i++) {
            Sample* s = sample(i);
            if (ret->prevValue > s->prevValue) {
                ret = s;
            }
        }
        return ret;
    }

    void Database::selectSamplesToReport(std::vector<Sample*>& report) {
        while(true) {
            Sample* ret = NULL;
            for (int i = 0; i < numSamples(); i++) {
                Sample* s = sample(i);
                if (reportedTags.find(s->tag) != reportedTags.end()) {
                    continue;
                }
                if (ret == NULL || ret->value > s->value) {
                    ret = s;
                }
            }
            if (ret == NULL) {
                break;
            }
            report.push_back(ret);
            reportedTags.insert(ret->tag);
            LOG(INFO) << FUNCTION_NAME() << " TAG = " << ret->tag;
        }

        reportedTags.erase(std::string("EXP"));
        
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    
    std::vector<bool> Database::parameterExplored() {
        std::vector<bool> ret;
        int dim = sample(0)->params.size();
        for (int i = 0; i < dim; i++) ret.push_back(false);

        for (int i = 0; i < numSamples(); i++) {
            Sample* s = sample(i);
            
            for (int j = 0; j < dim; j++) {
                ret[j] = (ret[j] | s->isParamUsed[j]);
            }
        }
        return ret;
    }
    
    void Database::sort() {
        std::sort( samples.begin(), samples.end(), betterValue );
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    void Database::sortByPrevValue() {
        std::sort( samples.begin(), samples.end(), betterPrevValue );
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    std::string Database::summary() {
        std::string msg =
            (boost::format("database: %d samples with best score %lf")
             % numSamples() % bestSample()->value
                ).str();
        return msg;
    }

    void Database::dump() {
        for (int i = 0; i < numSamples(); i++) {
            Sample* s = sample(i);
            cout << boost::format("%d: [%d] %s --> %lf(%lf) # %s")
                % i % s->id
                % IO(s->params) % s->value % s->prevValue
                % s->tag
                 << endl;
        }

        writeAsCSV();
    }

    void Database::writeAsCSV() {
        std::ofstream fout("dump.csv");
        int dim = sample(0)->params.size();

        for (int i = 0; i < dim; i++) {
        	char ch = 'a' + i;
        	if (i != 0) fout << ", ";
        	fout << ch;
        }
        fout << ", value";
        fout << ", prev";
        fout << ", tag";
        fout << ", id";
        fout << ", cls";
        fout << endl;
        
        for (int i = 0; i < numSamples(); i++) {
            Sample* s = sample(i);
            
            for (int j = 0; j < dim; j++) {
            	if (j != 0) fout << ", ";
                fout << s->params(j);
            }
            fout << ", " << s->value;
            fout << ", " << s->prevValue;
            fout << ", " << s->tag;
            fout << ", " << s->id;
            fout << ", " << s->debugClusterId;
            fout << endl;
        }
        fout.close();
    }

} // namespace mpisolver2
