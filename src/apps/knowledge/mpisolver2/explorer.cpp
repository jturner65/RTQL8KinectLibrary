#include "explorer.h"

#include <mpi.h>

#include "utils/UtilsMath.h"
#include "common/app_cppcommon.h"

#include "controller/SimPack.h"
#include "controller/controller"

#include "commandcenter.h"
#include "database.h"


namespace mpisolver2 {
    ////////////////////////////////////////////////////////////
    // class Explorer implementation
    Explorer::Explorer(CommandCenter* _cmd) 
        : MEMBER_INIT(cmd, _cmd)
        , MEMBER_INIT(isRunning, false)
        , MEMBER_INIT(startCount, 0)
    {
        
    }
    
    Explorer::~Explorer() {
    }

    void Explorer::incStartCount() {
        set_startCount( startCount() + 1);
    }

    void Explorer::start() {
        stop();
        set_isRunning(true);
        incStartCount();
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }
    
    void Explorer::stop() {

        set_isRunning(false);
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    void Explorer::mainloop() {
        if (!isRunning()) {
            return;
        }

        const int START = 1;
        char buff[MAX_PSM_BUFF_SIZE];

        std::vector<Sample*> workingSamples;
        for (int i = 0; i < START; i++) {
            workingSamples.push_back(NULL);
        }

        LOG(INFO) << FUNCTION_NAME();
        for (int id = START; id <= nChildProc(); id++) {
            const std::string NAME = (
                boost::format("[EXPThread %d/%d]") % id % nChildProc()
                ).str();

            Sample* s = this->generate();
            // s->tag = (boost::format("EXP%02d") % startCount()).str();
            s->tag = (boost::format("EXP")).str();
            LOG(INFO) << NAME << " sample genareated : " << IO(s->params);

            std::stringstream sout;
            for (int i = 0; i < s->params.size(); i++) {
                if (i != 0) sout << " ";
                sout << s->params(i);
            }

            sprintf(buff, "%s", sout.str().c_str());
            MPI_Send(&buff, MAX_PSM_BUFF_SIZE, MPI_PACKED, id, id, MPI_COMM_WORLD);
            LOG(INFO) << NAME << " message sent: wait for the response";

            workingSamples.push_back(s);
        }
        LOG(INFO) << FUNCTION_NAME() << " : ALL MESSAGE SENT!!!!!";
        
        for (int id = START; id <= nChildProc(); id++) {
            Sample* s = workingSamples[id];
            CHECK_NOTNULL(s);
            const std::string NAME = (
                boost::format("[EXPThread %d/%d]") % id % this->nChildProc()
                ).str();
            MPI_Status status;
            // cout << "wait for the response: " << src << " " << tag << endl;
            LOG(INFO) << NAME << " MPI_Recv...";
            MPI_Recv(&(s->value), 1, MPI_DOUBLE, id, id, MPI_COMM_WORLD, &status);
            s->prevValue = s->value;
            LOG(INFO) << NAME << " value = " << s->value;
            

            MPI_Recv(buff, MAX_PSM_BUFF_SIZE, MPI_PACKED, id, id, MPI_COMM_WORLD, &status);
            LOG(INFO) << NAME << " finalstate recv = " << strlen(buff) << endl;
            s->finalstate.fromString(buff);
//            cout << "Recved buffer = " << buff << endl;
//            cout << "===" << endl;
            // cout << "finalstate = " << s->finalstate << endl;

            LOG(INFO) << NAME << " add to DB...";
            db()->add(s);

        }
        
        LOG(INFO) << FUNCTION_NAME() << " pick and kill the victim...... # samples = " << db()->numSamples();
        int MAX_NUM_VICTIMS = 10;
        for (int i = 0; i < MAX_NUM_VICTIMS; i++) {
        	int v = pickVictim();
        	if (v == -1) {
        		break;
        	}
        	db()->erase(v);
        }
        LOG(INFO) << FUNCTION_NAME() << " pick and kill the victim done. # samples = " << db()->numSamples();

        LOG(INFO) << FUNCTION_NAME() << " OK";
        
    }

    int Explorer::nChildProc() {
        return cmd()->nChildProc();
    }
    controller::SimPack* Explorer::simpack() {
        return cmd()->simpack();
    }
    
    Database* Explorer::db() {
        return cmd()->database();
    }

    // 
    ////////////////////////////////////////////////////////////
        

    ////////////////////////////////////////////////////////////
    // class Explorer implementation
    NaiveExplorer::NaiveExplorer(CommandCenter* _cmd)
        : Explorer(_cmd)
    {
    }
    
    NaiveExplorer::~NaiveExplorer() {
    }
        
    Sample* NaiveExplorer::generate() {
        // boost::mutex::scoped_lock lock(mutex);
        const int dim = simpack()->con()->dim();
        Eigen::VectorXd params(dim);
        for (int i = 0; i < dim; i++) {
            params(i) = rtql8::utils::random(-1.0, 1.0);
        }

        Sample* s = new Sample(params);
        // s->params = params;
        return s;
    }
    
    int NaiveExplorer::pickVictim() {
        // boost::mutex::scoped_lock lock(mutex);
        return -1;
    }
    
    // 
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
     // class Explorer implementation
     DartThrowExplorer::DartThrowExplorer(CommandCenter* _cmd)
         : Explorer(_cmd)
     {
     }

     DartThrowExplorer::~DartThrowExplorer() {
     }

     Sample* DartThrowExplorer::generate() {
         const int dim = simpack()->con()->dim();
         const int MAX_TRIAL = 100;
         const double MIN_DIST = 0.0;
         const double MAX_DIST = 0.5;
         for (int i = 0; i < MAX_TRIAL; i++) {
        	 Sample* lhs = generateRandom();
        	 bool rejected = false;

             for (int j = 0; j < db()->numSamples(); j++) {
                 Sample* rhs = db()->sample(j);
                 double d = (lhs->params - rhs->params).norm();
                 double rad = CONFINE(rhs->value, MIN_DIST, MAX_DIST);
                 if (d < rad) {
                	 // Reject
                	 rejected = true;
                	 cout << "REJECT " << i << " try: sample " << IO(lhs->params) << " d = " << d << " rad = " << rad << endl;
                	 break;
                 }
             }
             if (rejected) {
            	 delete lhs;
             } else {
            	 cout << "ACCEPT " << i << " try: sample " << IO(lhs->params) << endl;
            	 return lhs;
             }
         }
    	 cout << "REACH THE MAX TRIAL" << endl;

         return generateRandom();
     }

     int DartThrowExplorer::pickVictim() {
         // boost::mutex::scoped_lock lock(mutex);
    	 const int MAX_CAPACITY = 10000;
    	 if (db()->numSamples() < MAX_CAPACITY) {
    		 return -1;
    	 }

    	 int retIndex = -1;
    	 Sample* ret = NULL;
         for (int i = 0; i < db()->numSamples(); i++) {
             Sample* s = db()->sample(i);
             if (ret == NULL || s->id < ret->id) {
            	 ret = s;
            	 retIndex = i;
             }
         }
         return retIndex;
     }

     Sample* DartThrowExplorer::generateRandom() {
         const int dim = simpack()->con()->dim();
         Eigen::VectorXd params(dim);
         for (int i = 0; i < dim; i++) {
             params(i) = rtql8::utils::random(-1.0, 1.0);
         }

         Sample* s = new Sample( params );
         // s->params = params;
         return s;
     }
     //
     ////////////////////////////////////////////////////////////

} // namespace mpisolver2
