#include "worker.h"

#include <sstream>

#include "utils/Paths.h"
#include "utils/UtilsRotation.h"

#include "toolkit/Toolkit.h"
#include "common/app_cppcommon.h"
#include "common/fullbody1.h"
#include "controller/controller"
#include <mpi.h>

#define MAX_DIM   20

namespace mpisolver {

    Worker::Worker(int _rank, int _numProcs)
        : MEMBER_INIT_NULL(sim)
        , MEMBER_INIT_NULL(con)
        , MEMBER_INIT(rank, _rank)
        , MEMBER_INIT(numProcs, _numProcs)
    {
        initialize();
        name = (boost::format("WORKER[%d]") % rank()).str();
    }
    
    Worker::~Worker() {
        destroy();
    }

    void Worker::initialize() {
        ////////////////////////////////////////////////////////////
        // Initialize Simulator
        rtql8::toolkit::SimConfig config(0.0005, 1.0, 0.0);
        config.skels.push_back(
            rtql8::toolkit::SkelConfig(RTQL8_DATA_PATH"/skel/ground1.skel", RTQL8_SKEL_IMMOBILE));
        config.skels.push_back(
            rtql8::toolkit::SkelConfig(RTQL8_DATA_PATH"/skel/fullbody2.skel"));
        set_sim( new rtql8::toolkit::Simulator(config) );

        // LOG(INFO) << "create simulator OK";
        cout << name << " : create simulator OK" << endl;

        ////////////////////////////////////////////////////////////
        // Initialize SimState
        rtql8::toolkit::SimState state = sim()->createEmptyState();

        state.skels[1].q <<
            0, 0.84, 0, 0, 0, 0, // Root dofs
            0.2, 0.1, -0.5, 0.3, 0, 0, // Left leg
            0.2, -0.1, -0.5, 0.3, 0, 0, // Right leg
            0.0, -0.25, 0, 0, 0, // Spine to Head
            0, 0, 0, 0, 0, // Left arm
            0, 0, 0, 0, 0; // Right arm
        sim()->setSimState(state);
        // LOG(INFO) << "set initial simstate OK";
        cout << name << " : set initial simstate OK" << endl;

        //////////////////////////////////////////////////////////////////////
        // Initialize  controller
        const int SID = 1;
        controller::XMLReader reader;
        controller::RootController* c = reader.read(RTQL8_DATA_PATH"xml/test.xml");
        c->setOwner(sim(), SID);
        set_con( c );
        // LOG(INFO) << FUNCTION_NAME() << " OK";
        cout << name << " : " << FUNCTION_NAME() << " OK" << endl;
    }

    void Worker::destroy() {
        cout << name << " : " << FUNCTION_NAME() << " OK" << endl;
        // LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    void Worker::mainloop() {
        MPI_Status status;
        const int BUFF_SIZE = 512;
        char buff[BUFF_SIZE];

        int src = 0;
        int tag = rank();

        const unsigned Dimension  = 2;


        while(true) {
            // printf("Wait for the message... [%d]\n", rank());
            MPI_Recv(buff, BUFF_SIZE, MPI_PACKED, src, tag, MPI_COMM_WORLD, &status);
            std::string cmd(buff);
            if (cmd == "QUIT") {
                // printf("QUIT child process [%d]\n", rank());
                cout << name << " : QUIT" << endl;
                break;
            }

            std::stringstream sin(buff);

            Eigen::VectorXd params(Dimension);
            int task = -1;
            for (unsigned i = 0; i < Dimension; i++) {
                sin >> params(i);
            }

            // LOG(INFO) << "[" << rank() << "] params = " << IO(params);
            con()->set_params( params );
            double ret = simLoop();
            // double ret = 0.01;

            cout << boost::format("%s : %.6lf <- %s") % name % ret  % IO(params) << endl;
            // cout << name << " : " << ret << " <- " << IO(params) << endl;

            // printf("process [%d] ", rank());
            // // printf("buff = [%s] ", buff);
            // for (unsigned i = 0; i < Dimension; i++) {
            //     printf(" %f", params(i) );
            // }
            // printf(" ret = [%lf] ", ret);
            // printf("\n");

            // MPI_Send(&ret, 1, MPI_FLOAT, src, rank, MPI_COMM_WORLD);
            MPI_Send(&ret, 1, MPI_DOUBLE, src, rank(), MPI_COMM_WORLD);
        }

        
    }

    double Worker::simLoop() {
        con()->reset();

        rtql8::toolkit::SimState saved = sim()->getSimState();
        // LOG(INFO) << rank() << " : " << "controller = " << endl << con()->phase()->toString() << endl;
        // LOG(INFO) << rank() << " : " << "initial state = " << sim()->getSimState();
        while(!con()->isTerminated()) {
            simStep();
            // LOG_EVERY_N(INFO, 100) << rank() << " : t = " << con()->t() << endl << con()->phase()->currentPhaseIndex();
        }
        double value = con()->evaluate();
        // LOG(INFO) << endl;
        // LOG(INFO) << rank() << " : " << "evaluate = " << con()->evaluate();
        // LOG(INFO) << rank() << " : " << "final state = " << sim()->getSimState();
        // LOG(INFO) << endl;

        sim()->setSimState(saved);
        return value;
    }
        

    void Worker::simStep() {
        if (con()->spd()) con()->spd()->pop();
        if (con()->vf())  con()->vf()->clear();
        
        // if (sim()->getSimState().nT() > 0) {
            con()->control();
        // }

        sim()->step();

    }


} // namespace mpisolver
