#include "optimizer.h"

#include "common/app_cppcommon.h"

#include "controller/SimPack.h"
#include "controller/controller"

#include "commandcenter.h"
#include "database.h"
#include "rcluster.h"
#include "boumancluster.h"
#include "MPICMASearch.h"

namespace mpisolver2 {

    Optimizer::Optimizer(CommandCenter* _cmd)
        : MEMBER_INIT(cmd, _cmd)
        , MEMBER_INIT(isRunning, false)
        , MEMBER_INIT(startCount, 0)
        , MEMBER_INIT(task, 0)
    {
    }
    
    Optimizer::~Optimizer() {
    }

    void Optimizer::deallocate() {
        LOG(INFO) << FUNCTION_NAME() << " : # cma searches = " << cmasearches.size();
        FOREACH(MPICMASearch* cma, cmasearches) {
            delete cma;
            
        }
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    void Optimizer::start() {
        stop();
        set_isRunning(true);
        set_startCount( startCount() + 1);


        deallocate();
        LOG(INFO) << FUNCTION_NAME() << " begins";
        // Rcluster rcluster(cmd());
        BoumanCluster cluster(cmd());
        cluster.cluster();

        for (int i = 0; i < cluster.numClasses(); i++) {
            set_task(i);
            // Start function
            // const int DIM  = 2;
            // MPIFunc f(DIM);
            MPIFunc f(this, cmd()->dim());
            Sample* best = cluster.bestSampleInClass(i);
            CHECK_NOTNULL(best);

            // Init start
            const double   MININIT        = -1.0;
            const double   MAXINIT        = 1.0;
            Array<double> start(cmd()->dim());
            start = Rng::uni(MININIT, MAXINIT);


            // start point
            MPICMASearch cma;
            Eigen::VectorXd m = cluster.classMean(i);
            Eigen::MatrixXd C = cluster.classCov(i);
            const double STEP = 1.0;
            cout << "dim = " << cmd()->dim() << endl;
            cout << "m = " << IO(m) << endl;
            cout << "C = " << endl << C << endl;
            cma.init(f, 6, 12, start, STEP, m, C);

            const double DESIRED = 0.005;
            // const double DESIRED = 0.5; // DEBUG
            if (best->value < DESIRED) {
                best->tag = (boost::format("OPT%02d%s")
                             % this->startCount() % this->taskName() ).str();

                LOG(INFO) << "**** TASK " << taskName() << " "
                          << "**** luckily, we already have our best solution!!!! "
                          << IO(best->params) << " : "
                          << best->value;
            } else {
                const int NITER = 5;
                for (int j = 0; j< NITER; j++)
                {
                    cma.run(nChildProc(), j);
                    LOG(INFO) << "**** TASK " << taskName() << " Iter " << j << " : "
                              << cma.bestSolutionFitness() << " ****";
                    if (cma.bestSolutionFitness() < DESIRED) {
                        break;
                    }
                }
            }
            LOG(INFO) << boost::format("********** TASK %s/%d has been finished!!! ************")
                % taskName() % cluster.numClasses();

        }

        LOG(INFO) << FUNCTION_NAME() << " OK";
    }
    
    void Optimizer::stop() {

        set_isRunning(false);
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    void Optimizer::mainloop() {
        if (!isRunning()) {
            return;
        }
        LOG_EVERY_N(INFO, 100) << FUNCTION_NAME() << " OK";
    }

    std::string Optimizer::taskName() {
        std::string ret(" ");
        ret[0] = (char)('A' + task());
        return ret;
    }

    int Optimizer::nChildProc() {
        return cmd()->nChildProc();
    }
    controller::SimPack* Optimizer::simpack() {
        return cmd()->simpack();
    }
    
    Database* Optimizer::db() {
        return cmd()->database();
    }
    

} // namespace mpisolver2
