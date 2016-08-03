#include <iostream>
using namespace std;

#include "worker.h"
using namespace server;

#ifdef KN_USE_MPI
#include <mpi.h>
#endif

// #include <boost/thread.hpp>

int main(int argc, char* argv[]) {
// #ifdef KN_DEPRESS_BOOST_LOG
//     cout << "KN_DEPRESS_BOOST_LOG is defined" << endl;
// #else
//     cout << "KN_DEPRESS_BOOST_LOG is NOT defined" << endl;
// #endif
//     return 0;

    // Worker wk(0);
    // wk.run();

    int numprocs, rank, namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];
  
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Get_processor_name(processor_name, &namelen);
    printf("Processor = [%s]\n", processor_name);
    printf("Process is %d/%d\n", rank, numprocs);
    // if (rank == 0) {
    //     Worker::initMutex();
    // } else {
    //     boost::posix_time::seconds t(1);  
    //     boost::this_thread::sleep(t);
    // }

    Worker wk(processor_name, rank);
    wk.run();
    MPI_Finalize();   
// #endif
}


