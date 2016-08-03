#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <mpi.h>
#include "commandcenter.h"
#include "worker.h"
#include "common/app_cppcommon.h"

void worker(int rank, int numProcs);
// void master(int rank, int numProcs);
void outputLog(const char* str);

int main(int argc, char* argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging((const char*)argv[0]);


    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = INFO;
    FLAGS_log_dir = "./glog/";


    srand( (unsigned int) time (NULL) );
    int numprocs, rank, namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];
  
    MPI_Init(&argc, &argv);
    // int provided = 0;
    // LOG(INFO) << "REQUESTED LEVEL = " << MPI_THREAD_MULTIPLE;
    // MPI_Init_thread(&argc, &argv, MPI_THREAD_MULTIPLE, &provided);
    // LOG(INFO) << "PROVIDED  LEVEL = " << provided;
    
    MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Get_processor_name(processor_name, &namelen);
    printf("Number of processes is %d\n", numprocs);


    if (rank == 0) {
        bool isLocal = false;
        for (int i = 0; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "local") {
                isLocal = true;
            }
        }

        mpisolver2::CommandCenter commandCenter( numprocs, isLocal );
        bool result = commandCenter.mainloop();

        // mpisolver::master(rank, numprocs);
    } else {
        worker(rank, numprocs);
    }

    printf("*************** Process %d on %s out of %d ****************************\n"
           , rank, processor_name, numprocs);

    MPI_Finalize();
}

void worker(int rank, int numProcs) {
    // printf("worker [%d/%d] started\n", rank, numProcs);
    LOG(INFO) << "worker " << rank << " started";
    mpisolver2::Worker worker(rank, numProcs);
    LOG(INFO) << "**** WORKER " << rank << " CLASS is successfully initialized ****";
    worker.mainloop();
    LOG(INFO) << "**** WORKER " << rank << " finished ****";

}

// void master(int rank, int numProcs) {
//     printf("master [%d] begins\n", rank);
// }

void outputLog(const char* str)
{
    std::ofstream log("log.txt", std::ios::app);
    log << str;
}
