/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SOLVER_REMOTEDB_H
#define SOLVER_REMOTEDB_H

#include <vector>
#include <string>
#include "common/app_hppcommon.h"

namespace sql {
    class Connection;
} // namespace sql

namespace controller {
    class Knowledge;
} // namespace controller

namespace solver {
    struct Sample;
} // namespace solver

namespace solver {
    
    class RemoteDB {
    public:
        RemoteDB();
        virtual ~RemoteDB();

        bool connect();
        int last_insert_id();
        bool registerKnowledge(controller::Knowledge* kn);


        void prepareMultipleInsertionQuery();
        int  executeMultipleInsertionQuery();

        
        bool deleteAllSamples();
        bool putSample(controller::Knowledge* kn, Sample* s, int index);
        bool getSampleFinal(controller::Knowledge* kn, Sample* s);

        int queryMultipleSamplesFinal(std::vector<Sample*> samples,
                                      std::vector<Sample*>* evaluated);

    protected:
        MEMBER_PTR(sql::Connection*, con);
        std::string kn_hash;
        std::string query;
        
    }; // class RemoteDB
    
} // namespace solver

#endif // #ifndef SOLVER_REMOTEDB_H

