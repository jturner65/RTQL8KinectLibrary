#include "master.h"
#include "MPICMASearch.h"
#include "common/app_cppcommon.h"
#include "cmaxmlwriter.h"
#include "cmacsvwriter.h"
#include "utils/Paths.h"

namespace mpisolver {
    void master(int rank, int numProcs) {
        printf("master [%d] begins\n", rank);

        Rng::seed(time(NULL) );

        //
        // fitness function
        //
        const unsigned Dimension  = 2;
        MPIFunc f(Dimension);

        //
        // EA parameters
        //
        // const unsigned Iterations     = 600;
        const unsigned Iterations     = 10;
        const double   MinInit        = .1;
        const double   MaxInit        = .3;
        // const double   GlobalStepInit = 1.;
        const double   GlobalStepInit = 0.8;

        // start point
        Array<double> start(Dimension);
        start = Rng::uni(MinInit, MaxInit);

        // search algorithm
        // CMASearch cma;
        MPICMASearch cma;
        CMAXMLWriter writer(&cma);
        CMACSVWriter writer2(&cma);
        
        // cma.init(f, 3, 6, start, GlobalStepInit);
        cma.init(f, 16, 32, start, GlobalStepInit);



        // optimization loop
        unsigned int i;
        for (i=0; i<Iterations; i++)
        {
            cma.run(numProcs, i);
            cout << i << " " << f.timesCalled() << " "  << cma.bestSolutionFitness() << endl;
            writer.addIteration(i);
            writer2.addIteration(i);

            // if (cma.bestSolutionFitness() < 0.001) {
            if (cma.bestSolutionFitness() < 0.00001) {
                // exit(EXIT_SUCCESS);
                break;
            }
            // break;
        }
        writer.write(RTQL8_DATA_PATH"/xml/result.xml");
        writer2.write("result.csv");

        // // lines below are for self-testing this example, please ignore
        // if (cma.bestSolutionFitness() < 10E-10) exit(EXIT_SUCCESS);
        // else exit(EXIT_FAILURE);
        cout << "master finished successfully" << endl;
        cma.kill(numProcs);

        writer.write(RTQL8_DATA_PATH"/xml/result.xml");

    }


} // namespace mpisolver
