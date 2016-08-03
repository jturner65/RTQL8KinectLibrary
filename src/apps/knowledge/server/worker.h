/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef SERVER_WORKER_H
#define SERVER_WORKER_H

#include <cstdio>
#include <string>
#include "common/app_hppcommon.h"
#include <boost/format.hpp>

namespace sql {
    class Connection;
} // namespace sql

namespace controller {
    class Knowledge;
} // namespace controller

namespace server {
    class Worker {
    public:
        Worker(const char* const _processor_name, int _rank);
        virtual ~Worker();

// Main functions
        bool connect();
        bool fetch();
        bool markSolved(int id);

        bool evaluate(int* pid);
        bool updateKnowledge(const char* const hash);
        void run();
        std::string statusFetch() { return name + "FETCH"; }
        std::string statusSolved() { return name + "SOLVED"; }
// 
        
    protected:
        int rank;
        std::string name;
        MEMBER_PTR(sql::Connection*, con);
        MEMBER_PTR(controller::Knowledge*, kn);
        std::string current_kn_hash;

// Synchronization
    protected:
        void initLog();
        void log(const boost::format& fmt);
        void log(const char* const txt);
        void flog(const boost::format& fmt);
        void flog(const char* const txt);
    protected:
        FILE* fp;
        
    }; // class Worker
    
} // namespace server

#endif // #ifndef SERVER_WORKER_H

