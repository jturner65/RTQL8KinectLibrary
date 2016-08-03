#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <mpi.h>
#include "master.h"
#include "worker.h"


void worker(int rank, int numProcs);
// void master(int rank, int numProcs);
void outputLog(const char* str);

int main(int argc, char* argv[])
{
    int numprocs, rank, namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];
  
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Get_processor_name(processor_name, &namelen);
    printf("Number of processes is %d\n", numprocs);
    if (rank == 0)
        mpisolver::master(rank, numprocs);
    else
        worker(rank, numprocs);

    //  printf("Process %d on %s out of %d\n", rank, processor_name, numprocs);

    MPI_Finalize();
}

void worker(int rank, int numProcs) {
    printf("worker [%d/%d] started\n", rank, numProcs);
    mpisolver::Worker worker(rank, numProcs);
    worker.mainloop();
    printf("worker [%d/%d] finished\n", rank, numProcs);
}

// void master(int rank, int numProcs) {
//     printf("master [%d] begins\n", rank);
// }

void outputLog(const char* str)
{
    std::ofstream log("log.txt", std::ios::app);
    log << str;
}
