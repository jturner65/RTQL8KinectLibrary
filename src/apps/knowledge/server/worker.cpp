/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "worker.h"
#include "utils/Timer.h"
#include "common/app_cppcommon.h"
using boost::format;
#include <sstream>
#include <boost/thread.hpp>

#include "mysql_connection.h"
#include "mysql_driver.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

// #define EXAMPLE_HOST "tcp://rdstest.ct0nmnwtwvkg.us-west-2.rds.amazonaws.com:3306"
#define EXAMPLE_HOST "tcp://127.0.0.1:3306"
#define EXAMPLE_USER "sehoonha"
#define EXAMPLE_PASS "siggraph2013"
#define EXAMPLE_DB "rdstestdb"

#include "toolkit/Toolkit.h"
#include "toolkit/Moreeigen.h"
using rtql8::toolkit::moreeigen::convertStringToVectorXd;

#include "controller/knowledge.h"
#include "controller/app_composite_controller.h"
#include "controller/simpack.h"

namespace server {
    
////////////////////////////////////////////////////////////
// class Worker implementation
    // Worker::Worker(int _rank)
    Worker::Worker(const char* const _processor_name, int _rank)
        : rank(_rank)
        , MEMBER_INIT_NULL(kn)
        , fp(NULL)
    {
        name = (boost::format("%s%02d") % _processor_name % rank).str();
        initLog();
    }
    
    Worker::~Worker() {
    }

// Main functions
    bool Worker::connect() {
	try {
            sql::SQLString url(EXAMPLE_HOST);
            sql::SQLString user(EXAMPLE_USER);
            sql::SQLString pass(EXAMPLE_PASS);
            sql::SQLString database(EXAMPLE_DB);

            // cout << "remotedb connect...." << endl;
            log("remote connect....");
            sql::Driver * driver = sql::mysql::get_driver_instance();
            /* Using the Driver to create a connection */
            sql::Connection*  con_ = driver->connect(url, user, pass);
            set_con(con_);
            // cout << "remotedb select schema...." << endl;
            log("remotedb select schema....");
            con()->setSchema(database);
            // cout << "remotedb connected to server!!!" << endl;
            log("remotedb connected to server!!!");
	} catch (sql::SQLException &e) {
            cerr << "# ERR: " << e.what();
            cerr << " (MySQL error code: " << e.getErrorCode();
            cerr << ", SQLState: " << e.getSQLState() << " )";
            exit(1);
            return false;
	}
        return true;
    }

    bool Worker::fetch() {
        using boost::format;
	try {
            {
                sql::Statement* stmt = con()->createStatement();
                std::stringstream sout;
                sout << "UPDATE rdstestdb.samples SET ";
                sout << format("status='%s' ") % statusFetch();
                sout << "WHERE status IS NULL ";
                sout << "LIMIT 1";
                std::string q = sout.str();
                flog(q.c_str());
                int result = stmt->executeUpdate(q.c_str());
                flog(format("... update result = %d") % result);
                if (result == 0) {
                    return false;
                }
            }
	} catch (sql::SQLException &e) {
            cerr << "# ERR: " << e.what();
            cerr << " (MySQL error code: " << e.getErrorCode();
            cerr << ", SQLState: " << e.getSQLState() << " )";
            exit(1);
            return false;
	}
        return true;
        
    }

    bool Worker::markSolved(int id) {
        using boost::format;
        std::string final = kn()->simpack()->sim()->getSimState().toString();
	try {
            {
                sql::Statement* stmt = con()->createStatement();
                std::stringstream sout;
                sout << "UPDATE rdstestdb.samples SET ";
                sout << format("status='%s' ") % statusSolved();
                sout << format(", final='%s' ") % final;
                sout << "WHERE id=" << id << " ";
                sout << "LIMIT 1";
                std::string q = sout.str();
                flog(q.c_str());
                int result = stmt->executeUpdate(q.c_str());
                flog(format("... result = %d") % result);

                if (result == 0) {
                    return false;
                }
                cout << "markSolved: id = " << id << endl;
            }
	} catch (sql::SQLException &e) {
            cerr << "# ERR: " << e.what();
            cerr << " (MySQL error code: " << e.getErrorCode();
            cerr << ", SQLState: " << e.getSQLState() << " )";
            exit(1);
            return false;
	}
        return true;
    }

    bool Worker::evaluate(int* pid) {
        using boost::format;
        using namespace controller;
	try {
            {
                sql::Statement* stmt = con()->createStatement();
                std::stringstream sout;
                sout << "SELECT * FROM rdstestdb.samples WHERE ";
                sout << format("status='%s' ") % statusFetch();
                sout << "LIMIT 1";
                std::string q = sout.str();
                cout << q << endl;
                std::auto_ptr< sql::ResultSet > res(stmt->executeQuery(q.c_str()));
                if (!res->next()) {
                    cerr << "sample is gone.. should not be reached.." << endl;
                    exit(1);
                } 
                // 0. Fetch variables
                int id = res->getInt("id");
                std::string current = res->getString("current").c_str();
                std::string kn_hash = res->getString("kn_hash").c_str();
                std::string nextrig = res->getString("nextrig").c_str();
                std::string params  = res->getString("params").c_str();

                *pid = id;
                flog(format("[ID = %d][Hash = %s]") % id % kn_hash);
                // cout << name << " : " << id << " = [" << kn_hash << "]" << endl;
                // 1. update the knowledge up-to-date
                updateKnowledge(kn_hash.c_str());

                // 2. set the current controller
                // cout << "current = " << current << endl;
                AppCompositeController* con = dynamic_cast<AppCompositeController*>(
                    kn()->getControllerDup(current.c_str())
                    );

                // 3. set the nextrig
                flog(format("nextrig = {%s}") % nextrig);
                // cout << "nextrig = [" << nextrig << "]" << endl;
                if (nextrig.length() > 0) {
                    Rig* rig = (Rig*)kn()->getControllerDup(nextrig.c_str());
                    if (rig == NULL) {
                        cerr << "rig is NULL. why is that?" << endl;
                        exit(1);
                    }
                    con->addRig(rig);
                    // cout << "rig is added" << endl;
                    flog("rig is added");
                }

                // 4. Set the parameter
                // flog("I'm here");
                // flog(format("params.str = ") % params);
                Eigen::VectorXd paramsvx = convertStringToVectorXd(params.c_str());
                // flog("params.fetched.");
                // exit(0);
                // cout << "params.vx = " << IO(paramsvx) << endl;
                con->setParams(paramsvx);

                SimPack* sp = kn()->simpack();
                sp->set_con(con);
                sp->reset();
                sp->simulate();
                
                // LOG_INFO << "evaluate OK";
                flog("evaluate OK");
            }
	} catch (sql::SQLException &e) {
            cerr << "# ERR: " << e.what();
            cerr << " (MySQL error code: " << e.getErrorCode();
            cerr << ", SQLState: " << e.getSQLState() << " )";
            exit(1);
            return false;
	}
        return true;
    }

    bool Worker::updateKnowledge(const char* const hash) {
        if (current_kn_hash == std::string(hash)) {
            return true;
        }
        using boost::format;
	try {
            {
                sql::Statement* stmt = con()->createStatement();
                std::stringstream sout;
                sout << "SELECT * FROM rdstestdb.knowledge WHERE ";
                sout << format("hash='%s' ") % hash;
                sout << "LIMIT 1";
                std::string q = sout.str();
                flog(q.c_str());

                std::auto_ptr< sql::ResultSet > res(stmt->executeQuery(q.c_str()));
                if (!res->next()) {
                    cerr << FUNCTION_NAME() << " : invalid hash = " << hash;
                    exit(0);
                }
                // cout << "ok." << endl;
                std::string xmlstring = res->getString("xmlstring").c_str();
                // cout << "xmlstring = " << xmlstring << endl;
                if (kn()) delete kn();
                controller::Knowledge* mykn = new controller::Knowledge();
                mykn->turnOffLog();
                mykn->loadXMLString(xmlstring.c_str());
                set_kn( mykn );
                current_kn_hash = hash;

                // This may important, so log both std and file
                log(format("... new hash = %s") % current_kn_hash);
                // cout << "new hash = " << current_kn_hash << endl;
                // cout << "mykn = " << mykn->toString() << endl;
            }
	} catch (sql::SQLException &e) {
            cerr << "# ERR: " << e.what();
            cerr << " (MySQL error code: " << e.getErrorCode();
            cerr << ", SQLState: " << e.getSQLState() << " )";
            exit(1);
            return false;
	}
    }

    void Worker::run() {
        connect();
        log("connected!");
        // cout << "[" << name << "] connected!";
        // const int MAX_ITER = 10;
        using boost::format;
        rtql8::utils::Timer timer("TIMER");
        for (int i = 0; ; i++) {
            // cout << endl << "[" << name << "] start to fetch a new sample" << endl;
            log("fetch a new sample");
            bool result = fetch();
            if (!result) {
                log("sleep()...");
                boost::posix_time::milliseconds t(333);  
                boost::this_thread::sleep(t);
                continue;
            }
            int sid = -1;
            timer.startTimer();
            log("fetched");
            result = evaluate(&sid);
            if (!result) break;
            markSolved(sid);
            timer.stopTimer();
            log(format("solved %.4lf (cnt: %d, avg:%.4lf")
                % timer.lastElapsed() % timer.getCount() % timer.getAve());
        }
        log("run() FINISHED!!");
        // cout << FUNCTION_NAME() << " OK" << endl;
    }

// Synchronization


    void Worker::initLog() {
        // mtx = new boost::mutex();
        // slog((boost::format("%s OK") % FUNCTION_NAME()).str().c_str());
        fp = fopen((format("log_%02d.txt") % rank).str().c_str(), "w+");
    }

    void Worker::log(const boost::format& fmt) {
        log(fmt.str().c_str());
    }


    void Worker::log(const char* const txt) {
        printf("[%s] %s\n", name.c_str(), txt);
        fprintf(fp, "[%s] %s\n", name.c_str(), txt);
        fflush(fp);
    }

    void Worker::flog(const boost::format& fmt) {
        flog(fmt.str().c_str());
    }

    void Worker::flog(const char* const txt) {
        fprintf(fp, "[%s] %s\n", name.c_str(), txt);
        fflush(fp);
    }

// class Worker ends
////////////////////////////////////////////////////////////
    
    
} // namespace server



