#include "worker.h"

#include <sstream>

#include "utils/Paths.h"
#include "utils/UtilsRotation.h"

#include "toolkit/Toolkit.h"
#include "common/app_cppcommon.h"
#include "common/fullbody1.h"
#include "controller/SimPack.h"
#include "controller/controller"
#include <mpi.h>
#include "sample.h"

#define MAX_DIM   20

namespace mpisolver2 {

    Worker::Worker(int _rank, int _numProcs)
        : MEMBER_INIT_NULL(simpack)
        , MEMBER_INIT(rank, _rank)
        , MEMBER_INIT(numProcs, _numProcs)
    {
        initialize();
        name = (boost::format(" WORKER[%d]") % rank()).str();
    }
    
    Worker::~Worker() {
        destroy();
    }

    void Worker::initialize() {
        set_simpack( new controller::SimPack() );
        // simpack()->init(RTQL8_DATA_PATH"xml/temp.xml"); // Should be modified
        simpack()->init(); // Should be modified

        LOG(INFO) << FUNCTION_NAME() << name << " : OK";
    }

    void Worker::destroy() {
        LOG(INFO) << FUNCTION_NAME() << name << " : OK";
    }

    void Worker::mainloop() {
        MPI_Status status;
        char buff[MAX_PSM_BUFF_SIZE];

        int master = 0;
        // int src = rank();
        int tag = rank();

//        const unsigned Dimension  = 2;


        while(true) {
            // printf("Wait for the message... [%d]\n", rank());
            MPI_Recv(buff, MAX_PSM_BUFF_SIZE, MPI_PACKED, master, tag, MPI_COMM_WORLD, &status);
            // printf("Message recieved... [%d]\n", rank());
            std::string cmd(buff);

            // LOG(INFO) << name << " cmd = " << cmd;
            cout << name << " cmd = " << cmd << endl;

            if (cmd == "QUIT") {
                // printf("QUIT child process [%d]\n", rank());
                cout << name << " : QUIT" << endl;
                break;
            } else if (cmd == "newcontroller") {
                onNewController();
                continue;
            }

            std::stringstream sin(buff);

            int DIM = simpack()->con()->dim();
            Eigen::VectorXd params(DIM);
            int task = -1;
            for (unsigned i = 0; i < DIM; i++) {
                sin >> params(i);
            }
            cout << "param = " << IO(params) << endl;

            simpack()->con()->set_params( params );
            double ret = simLoop();

            LOG(INFO) << boost::format("%s : %.6lf <- %s") % name % ret  % IO(params);
            MPI_Send(&ret, 1, MPI_DOUBLE, master, tag, MPI_COMM_WORLD);

            controller::PhaseStateMap final;
            simpack()->con()->phase()->getPhaseStateMap(&final);

//            LOG(INFO) << "PhaseStateMap = " << endl << final.toString();

          //  rtql8::toolkit::SimState& final = simpack()->sim()->getSimState();
            sprintf(buff, "%s", final.toString().c_str());
            // cout << "finalstate sent = " << strlen(buff) << endl;
            // cout << "finalstate = " << final << endl;
            MPI_Send(&buff, MAX_PSM_BUFF_SIZE, MPI_PACKED, master, tag, MPI_COMM_WORLD);
        }

        
    }

    double Worker::simLoop() {
        // printf("reset simulation... [%d]\n", rank());

        simpack()->reset();
//        printf("start simulation... [%d]\n", rank());
        simpack()->simulate();
//        printf("end simulation... [%d]\n", rank());
        double value = simpack()->con()->evaluate();
        return value;
    }

    void Worker::onNewController() {
        using boost::format;
        cout << FUNCTION_NAME() << name << endl;
        MPI_Status status;
        const int BUFF_SIZE = 2048;
        char buff[BUFF_SIZE];
        int master = 0;
        int tag = rank();
        cout << FUNCTION_NAME() << name <<
            format("Wait for the message... [%d]\n") % rank() << endl;
        MPI_Recv(buff, BUFF_SIZE, MPI_PACKED, master, tag, MPI_COMM_WORLD, &status);
        std::string filename(buff);
        cout << FUNCTION_NAME() << name <<
            filename << endl;

        simpack()->initController(filename.c_str());
        cout << FUNCTION_NAME() << name << " : OK" << endl;
    }

} // namespace mpisolver2
