#ifndef MPISOLVER2_DATABASE_H
#define MPISOLVER2_DATABASE_H

#include <set>
#include <vector>
#include "sample.h"

namespace mpisolver2 {

    class Database {
    public:
        Database();
        virtual ~Database();

        void clear();
        void add(Sample* const s);
        bool erase(int index);
        int numSamples() const { return samples.size(); }
        Sample* sample(int index) { return samples[index]; }
        Sample* bestSample();
        Sample* bestSampleByPrev();

        void selectSamplesToReport(std::vector<Sample*>& report);
        std::vector<bool> parameterExplored();
        void sort();
        void sortByPrevValue();
        std::string summary();

        void dump();

        void writeAsCSV();
        
    protected:
        std::vector<Sample*> samples;
        std::set<std::string> reportedTags;
        
    };

    

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_DATABASE_H
