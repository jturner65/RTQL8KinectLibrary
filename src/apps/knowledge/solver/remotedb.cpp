/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "remotedb.h"

#include <sstream>

#include "common/app_cppcommon.h"

#include "mysql_connection.h"
#include "mysql_driver.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

// #define EXAMPLE_HOST "tcp://rdstest.ct0nmnwtwvkg.us-west-2.rds.amazonaws.com:3306"
// #define EXAMPLE_HOST "tcp://127.0.0.1:3306"
#define EXAMPLE_HOST "localhost"
#define EXAMPLE_USER "sehoonha"
#define EXAMPLE_PASS "siggraph2013"
#define EXAMPLE_DB "rdstestdb"

#include "controller/knowledge.h"
#include "sample.h"

namespace solver {
    
////////////////////////////////////////////////////////////
// class RemoteDB implementation
    RemoteDB::RemoteDB() {
    }
    
    RemoteDB::~RemoteDB() {
    }

    bool RemoteDB::connect() {
	try {
            sql::SQLString url(EXAMPLE_HOST);
            sql::SQLString user(EXAMPLE_USER);
            sql::SQLString pass(EXAMPLE_PASS);
            sql::SQLString database(EXAMPLE_DB);

            LOG_INFO << "remotedb connect....";
            sql::Driver * driver = sql::mysql::get_driver_instance();
            /* Using the Driver to create a connection */
            sql::Connection*  con_ = driver->connect(url, user, pass);
            set_con(con_);
            LOG_INFO << "remotedb select schema....";
            con()->setSchema(database);
            LOG_INFO << "remotedb connected to server!!!";
	} catch (sql::SQLException &e) {
            LOG_ERROR << "# ERR: " << e.what();
            LOG_ERROR << " (MySQL error code: " << e.getErrorCode();
            LOG_ERROR << ", SQLState: " << e.getSQLState() << " )";
            exit(1);
            return false;
	}

        return true;
    }

    int RemoteDB::last_insert_id() {

        std::auto_ptr< sql::Statement > stmt(con()->createStatement());
        std::string q = "SELECT last_insert_id()";
        std::auto_ptr< sql::ResultSet > res(stmt->executeQuery(q.c_str()));
        int ret = -1;
        if (res->next()) {
            ret = res->getInt("last_insert_id()");
        }
        return ret;
    }

    bool RemoteDB::registerKnowledge(controller::Knowledge* kn) {
	try {
            std::string hash = kn->hashString();
            this->kn_hash = hash;
            LOG_INFO << "hash = [" << hash << "]";
            {
                std::auto_ptr< sql::Statement > stmt(con()->createStatement());
                std::string q = (boost::format(
                    "SELECT * FROM rdstestdb.knowledge WHERE hash='%s'") % hash).str();
                LOG_INFO << "querying the knowledge..";
                LOG_INFO << q;
                std::auto_ptr< sql::ResultSet > res(stmt->executeQuery(q.c_str()));
                LOG_INFO << "...";
                
                if (res->next()) {
                    LOG_INFO << "we already have the same knowlege in the database!!";
                    return true;
                }
            }
            LOG_INFO << "we do not have knowlege in the database: putting it..";
            std::string xmlstring = kn->saveXMLString();
            {
                sql::Statement* stmt = con()->createStatement();
                std::string q = (boost::format(
                    "INSERT INTO rdstestdb.knowledge (hash, xmlstring) VALUES ('%s', '%s')") % hash % xmlstring).str();
                LOG_INFO << "inserting the knowledge..";
                LOG_INFO << q.substr(0, 100);
                int result = stmt->executeUpdate(q.c_str());
                LOG_INFO << "... result = " << result;
            }
            LOG_INFO << "insertion OK";
            
	} catch (sql::SQLException &e) {
            LOG_INFO << "# ERR: " << e.what()
                     << " (MySQL error code: " << e.getErrorCode()
                     << ", SQLState: " << e.getSQLState() << " )";
            return false;
	}
        return true;
    }

    void RemoteDB::prepareMultipleInsertionQuery() {
        std::stringstream sout;
        sout << "INSERT INTO rdstestdb.samples (kn_hash, current, nextrig, ";
        sout << "params) VALUES";
        query = sout.str();
    }

    int RemoteDB::executeMultipleInsertionQuery() {
	try {
            LOG_INFO << "putting Samples...";
            {
                sql::Statement* stmt = con()->createStatement();
                std::string q = this->query;
                LOG_INFO << "updating the database...";
                LOG_INFO << q;
                int result = stmt->executeUpdate(q.c_str());
                LOG_INFO << "... result = " << result;
            }
            int ret = last_insert_id();
            LOG_INFO << "insertion OK: dbid = " << ret;
            return ret;
	} catch (sql::SQLException &e) {
            LOG_INFO << "# ERR: " << e.what()
                     << " (MySQL error code: " << e.getErrorCode()
                     << ", SQLState: " << e.getSQLState() << " )";
            return -1;
	}
        return -1;
    }

    bool RemoteDB::deleteAllSamples() {
	try {
            LOG_INFO << "delete all samples...";
            {
                sql::Statement* stmt = con()->createStatement();
                std::string q = "TRUNCATE rdstestdb.samples";
                LOG_INFO << q;
                int result = stmt->executeUpdate(q.c_str());
            }
            LOG_INFO << "delete all samples... OK";
	} catch (sql::SQLException &e) {
            LOG_INFO << "# ERR: " << e.what()
                     << " (MySQL error code: " << e.getErrorCode()
                     << ", SQLState: " << e.getSQLState() << " )";
            return false;
	}
        return true;
    }

    bool RemoteDB::putSample(controller::Knowledge* kn, Sample* s, int index) {
        using boost::format;
        std::stringstream sout;
        if (index != 0) {
            sout << ", ";
        }
        sout << "(";
        sout << format("'%s'") % (this->kn_hash);
        sout << format(", '%s'") % (s->current);
        sout << format(", '%s'") % (s->nextrig);
        sout << ", '" << IO(s->params) << "'";
        sout << ")";
        std::string q = sout.str();
        query += q;
        return true;
    }

    bool RemoteDB::getSampleFinal(controller::Knowledge* kn, Sample* s) {
	try {
            {
                std::auto_ptr< sql::Statement > stmt(con()->createStatement());
                std::stringstream sout;
                sout << "SELECT * FROM rdstestdb.samples WHERE ";
                sout << "id = " << s->dbid <<" AND ";
                sout << "final IS NOT NULL ";
                sout << "LIMIT 1";
                std::string q = sout.str();

                // LOG_INFO << "querying the sample final state.. [" << s->dbid << "]";
                // LOG_INFO << q;
                std::auto_ptr< sql::ResultSet > res(stmt->executeQuery(q.c_str()));
                // LOG_INFO << "...";
                
                if (res->next()) {
                    LOG_INFO << "found the sample final state.. [" << s->dbid << "]";
                    s->final = res->getString("final").c_str();
                    // LOG_INFO << "we found the final state! length = " << s->final.length();
                    return true;
                } else {
                    // LOG_INFO << "we cannot find the final state";
                    return false;
                }
            }
	} catch (sql::SQLException &e) {
            LOG_INFO << "# ERR: " << e.what()
                     << " (MySQL error code: " << e.getErrorCode()
                     << ", SQLState: " << e.getSQLState() << " )";
            return false;
	}
        return false;
    }

    int RemoteDB::queryMultipleSamplesFinal(std::vector<Sample*> samples,
                                            std::vector<Sample*>* evaluated) {
	try {
            {
                std::auto_ptr< sql::Statement > stmt(con()->createStatement());
                std::stringstream sout;
                sout << "SELECT * FROM rdstestdb.samples WHERE ";
                sout << "id IN (";
                for (int i = 0; i < samples.size(); i++) {
                    Sample* s = samples[i];
                    if (i != 0) sout << ", ";
                    sout << s->dbid;
                }
                sout << ") AND ";
                sout << "final IS NOT NULL ";
                std::string q = sout.str();

                // LOG_INFO << "queryMultipleSamplesFinal:: " << endl << q;
                std::auto_ptr< sql::ResultSet > res(stmt->executeQuery(q.c_str()));
                // LOG_INFO << "...";

                int answeredCount = 0;
                while(res->next()) {
                    int dbid = res->getInt("id");
                    // LOG_INFO << "fetch result: " << dbid;
                    std::string final = res->getString("final").c_str();
                    Sample* s = NULL;
                    FOREACH(Sample* rhs, samples) {
                        if (rhs->dbid == dbid) {
                            s = rhs;
                            break;
                        }
                    }
                    if (s) {
                        s->final = final;
                        evaluated->push_back(s);
                        answeredCount++;
                    }
                }
                return answeredCount;
            }
	} catch (sql::SQLException &e) {
            LOG_INFO << "# ERR: " << e.what()
                     << " (MySQL error code: " << e.getErrorCode()
                     << ", SQLState: " << e.getSQLState() << " )";
            return -1;
	}
        return -1;
    }

// class RemoteDB ends
////////////////////////////////////////////////////////////
    
    
} // namespace solver



