#include "boumancluster.h"

extern "C" {
#include "cluster/clust_defs.h"
#include "cluster/alloc_util.h"
#include "cluster/clust_io.h"
#include "cluster/clust_util.h"
#include "cluster/subcluster.h"
#include "cluster/classify_util.h"
}

#include <algorithm>
#include <fstream>
#include "utils/UtilsMath.h"
#include "common/app_cppcommon.h"


#include "commandcenter.h"
#include "database.h"

namespace mpisolver2 {

    BoumanCluster::BoumanCluster(CommandCenter* _cmd)
        : MEMBER_INIT(cmd, _cmd)
    {
    }
    
    BoumanCluster::~BoumanCluster() {
    }

    void BoumanCluster::cluster() {
        goodsamples.clear();
        cls.clear();
        means.clear();
        covs.clear();

        // select();
        // selectByFixedNumber();
        selectByRatio();
        cout << "run().." << endl;
        run();
        cout << "cluster()..OK" << endl;
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    void BoumanCluster::selectByFixedNumber() {
        cout << "selectByFixedNumber().." << endl;
        // Select 33% of sampels
        db()->sortByPrevValue();
        int n = db()->numSamples() / 3;
        // n = std::min(n, 16);
        
        for (int i = 0; i < n; i++) {
            Sample* s = db()->sample(i);
            goodsamples.push_back(s);
        }
        LOG(INFO) << FUNCTION_NAME() << " : the number of selected samples = " << n;
    }

    void BoumanCluster::selectByRatio() {
        const double ratio = 2.0;

        bool usePrev = false;
        Sample* bestSample = db()->bestSample();
        CHECK_NOTNULL(bestSample);
        const double USE_PREV_CRITERIA = 0.3;
        if (bestSample->value > USE_PREV_CRITERIA) {
            LOG(INFO) << FUNCTION_NAME()
                      << "USE_PREV = true because bestValue = " << bestSample->value;
            usePrev = true;
        } else {
            LOG(INFO) << FUNCTION_NAME()
                      << "USE_PREV = false because bestValue = " << bestSample->value;
            usePrev = false;
        }

        
        
        if (!usePrev) {
            db()->sort();
        } else{
            db()->sortByPrevValue();
        }
            
        // double minValue = db()->sample(0)->prevValue;

        std::vector<bool> explored = db()->parameterExplored();
        int n = cmd()->dim();
        int m = 0;
        for (int i = 0; i < explored.size(); i++) {
            if (explored[i]) m++;
        }
        int DIM = m;

        int NPARAMS_CLUST = 1 + DIM + 0.5 * (DIM + 1) * DIM;
        int NCLUST = 2;
        int minSamples = NPARAMS_CLUST * NCLUST;

        Sample* first = db()->sample(0);
        double minValue =  usePrev ? first->prevValue : first->value;
        double upperBound = ratio * minValue;
        if (upperBound < 0.005) {
            upperBound = 0.005;
        }
        cout << "selectByRatio().." << endl;
        LOG(INFO) << FUNCTION_NAME()
                  << " minValue = " << minValue
                  << " upperBound = " << upperBound
                  << " minSamples = " << minSamples;

        for (int i = 0; i < db()->numSamples(); i++) {
            Sample* s = db()->sample(i);
            double v = usePrev ? s->prevValue : s->value;
            if (v > upperBound && i > minSamples) {
                break;
            }
            goodsamples.push_back(s);
        }
        
        LOG(INFO) << FUNCTION_NAME()
                  << " : the number of selected samples = " << goodsamples.size();
    }

    double AverageVariance(struct ClassSig *Sig, int nbands)
    {
        int     i,b1;
        double  *mean,**R,Rmin;

        /* Compute the mean of variance for each band */
        mean = G_alloc_vector(nbands);
        R = G_alloc_matrix(nbands,nbands);

        for(b1=0; b1<nbands; b1++) {
            mean[b1] = 0.0;
            for(i=0; i<Sig->ClassData.npixels; i++) {
                mean[b1] += (Sig->ClassData.x[i][b1])*(Sig->ClassData.w[i]);
            }
            mean[b1] /= Sig->ClassData.SummedWeights;
        }

        for(b1=0; b1<nbands; b1++) {
            R[b1][b1] = 0.0;
            for(i=0; i<Sig->ClassData.npixels; i++) {
                R[b1][b1] += (Sig->ClassData.x[i][b1])*(Sig->ClassData.x[i][b1])*(Sig->ClassData.w[i]);
            }
            R[b1][b1] /= Sig->ClassData.SummedWeights;
            R[b1][b1] -= mean[b1]*mean[b1];
        }

        /* Compute average of diagonal entries */
        Rmin = 0.0;
        for(b1=0; b1<nbands; b1++) 
            Rmin += R[b1][b1];

        Rmin = Rmin/(nbands);

        G_free_vector(mean);
        G_free_matrix(R);

        return(Rmin);
    }

    void BoumanCluster::run() {
        int NCLASSES = 1; // The number of class before doing clustering: we have only one
        int NINITSUBCLASSES = 2; // The number of initial subclasses (results): maybe adjusted

        clusterMessageVerboseLevel = 2;
        
        std::vector<bool> explored = db()->parameterExplored();
        int n = cmd()->dim();
        int m = 0;
        for (int i = 0; i < explored.size(); i++) {
            if (explored[i]) m++;
            cout << "dim " << i << " explored = " << explored[i] << endl;
        }
        cout << "Original dimension: " << n << " Explored dimension: " << m << endl;
        int DIM = m;


        SigSet S;
        memset(&S, 0, sizeof(SigSet));
        /* Initialize SigSet data structure */
        I_InitSigSet (&S);
        I_SigSetNBands (&S, DIM);
        I_SetSigTitle (&S, (char*)"BoumanCluster signature set");

        /* Allocate memory for cluster signatures */
        for (int k = 0; k < NCLASSES; k++) {
            ClassSig* Sig = I_NewClassSig(&S);
            I_SetClassTitle (Sig, (char*)"test class signature");
            for(int i = 0; i < NINITSUBCLASSES; i++) {
                I_NewSubSig (&S, Sig);
            }
        }

        int NSAMPLES = goodsamples.size();
        /* Read data for each class */
        for (int k = 0; k < NCLASSES; k++) {
            struct ClassSig* Sig = &(S.ClassSig[k]);
            I_AllocClassData (&S, Sig, NSAMPLES);

            /* Read Data */
            for(int i = 0; i < Sig->ClassData.npixels; i++) {
                const Eigen::VectorXd& gene = goodsamples[i]->reducedParam( explored );
                for(int j = 0; j < DIM; j++) {
                    Sig->ClassData.x[i][j] = gene(j);
                }
            }

            /* Set unity weights and compute SummedWeights */
            Sig->ClassData.SummedWeights = 0.0;
            for(int i = 0; i < Sig->ClassData.npixels; i++) {
                Sig->ClassData.w[i] = 1.0;
                Sig->ClassData.SummedWeights += Sig->ClassData.w[i];
            }
        }
        
        /* Compute the average variance over all classes */
        double Rmin = 0;
        for(int k = 0; k < NCLASSES; k++) {
            struct ClassSig* Sig = &(S.ClassSig[k]);
            Rmin += AverageVariance(Sig, DIM);
        }
        Rmin = Rmin/(COVAR_DYNAMIC_RANGE * NCLASSES);

        // LOG(INFO) << "try to save sigset...";
        // FILE* fp = fopen("sigset.txt", "w+");
        // I_WriteSigSet(fp, &S);
        // fclose(fp);
        // LOG(INFO) << "try to save sigset...OK";
        // cout << "try to save sigset OK!!!!!!!!";


        for(int k = 0; k < NCLASSES; k++) {
            int max_num;
            int option2 = 0;
            struct ClassSig* Sig = &(S.ClassSig[k]);
            if(1<=clusterMessageVerboseLevel) {
                fprintf(stdout,"Start clustering class %d\n\n",k);
            }

            subcluster(&S,k,option2,(int)CLUSTER_FULL,Rmin,&max_num);

            if(2<=clusterMessageVerboseLevel) {
                fprintf(stdout,"Maximum number of subclasses = %d\n",max_num);
            }

            I_DeallocClassData(&S, Sig);
        }

        /* read the result and parse into class members */
        int nsubclasses = 0;
        {
            struct ClassSig* Cp = &(S.ClassSig[0]);
            nsubclasses = Cp->nsubclasses;
            for (int j = 0; j < Cp->nsubclasses; j++) {
                struct SubSig* Sp = &Cp->SubSig[j];

                Eigen::VectorXd m( DIM );
                for (int q = 0; q < DIM; q++) {
                    m(q) = Sp->means[q];
                }
                m = expand(m, explored);
                means.push_back(m);

                Eigen::MatrixXd C( DIM, DIM );
                for (int q = 0; q < DIM; q++) {
                    for (int w = 0; w < DIM; w++) {
                        C(q, w) = Sp->R[q][w];
                    }
                }
                C = expand(C, explored);
                covs.push_back(C);
                
                LOG(INFO) << "Class " << j;
                LOG(INFO) << "Mean = " << IO(m);
                LOG(INFO) << "Cov = " << endl << C;
            }

        }
        LOG(INFO) << "Result = " << nsubclasses;

        ////////////////////////////////////////////////////////////
        // To classify each sample, we need to flatten the structure(SplitClasses.c)
        struct SigSet& Sin = S;
        struct SigSet Sout;
        /* Initialize SigSet data structure */
        I_InitSigSet (&Sout);
        I_SigSetNBands (&Sout, Sin.nbands);
        I_SetSigTitle (&Sout, (char*)"signature set for unsupervised clustering");

        /* Copy each subcluster (subsignature) from input to cluster (class signature) of output */
        for(int k = 0; k < Sin.nclasses; k++) {
            for(int l = 0; l < (Sin.ClassSig[k].nsubclasses); l++) {
                struct ClassSig* Sig = I_NewClassSig(&Sout);
                I_SetClassTitle (Sig, (char*)"Single Model Class");
                I_NewSubSig (&Sout, Sig);
                Sig->SubSig[0].pi = 1.0;
                for(int i = 0; i < Sin.nbands; i++) {
                    Sig->SubSig[0].means[i] = Sin.ClassSig[k].SubSig[l].means[i];
                }
                for(int i = 0; i < Sin.nbands; i++) {
                    for(int j = 0; j < Sin.nbands; j++) {
                        Sig->SubSig[0].R[i][j] = Sin.ClassSig[k].SubSig[l].R[i][j];
                    }
                }
            }
        }
        LOG(INFO) << "Flatten OK";
        
        /* Initialize constants for Log likelihood calculations */
        ClassLogLikelihood_init(&Sout);

        /* Clear the cluster Id of samples */
        for (int i = 0; i < db()->numSamples(); i++) {
            Sample* s = db()->sample(i);
            s->debugClusterId = -1;
        }        
        
        /* Compute Log likelihood for each class*/
        double* ll = G_alloc_vector(S.nclasses);
        double* data = new double[DIM + 1];
        for(int i = 0; i < goodsamples.size(); i++) {
            Sample* gs = goodsamples[i];
            const Eigen::VectorXd& gene = gs->reducedParam( explored );
            for (int j = 0; j < DIM; j++) {
                data[j] = gene(j);
            }
            ClassLogLikelihood(data,ll,&Sout); 
            double maxval = ll[0];
            int maxindex = 0;
            for(int j = 0; j < Sout.nclasses; j++) {
                if( ll[j] > maxval ) {
                    maxval = ll[j];
                    maxindex = j;
                }
            }
            gs->debugClusterId = maxindex;
            cls.push_back(maxindex);
            LOG(INFO) << "Sample " << i << " " << IO(gs->params)
                      << " : class = " << maxindex
                      << " (" << gs->value << " / " << gs->prevValue << ")";

        }
        delete[] data;

        I_DeallocSigSet(&S); 

        LOG(INFO) << FUNCTION_NAME() << " OK";

    }

    Eigen::VectorXd BoumanCluster::expand(const Eigen::VectorXd& v,
                                          const std::vector<bool>& explored) {
        int n = cmd()->dim();
        int m = 0;
        for (int i = 0; i < explored.size(); i++) { if (explored[i]) m++; }

        Eigen::VectorXd ret = Eigen::VectorXd::Zero(n);
        int ptr = 0;
        for (int i = 0; i < n; i++) {
            if (!explored[i]) continue;
            ret(i) = v(ptr++);
        }
        CHECK_EQ(ptr, m);
        return ret;

    }

    Eigen::MatrixXd BoumanCluster::expand(const Eigen::MatrixXd& M,
                                          const std::vector<bool>& explored) {
        int n = cmd()->dim();
        int m = 0;
        for (int i = 0; i < explored.size(); i++) { if (explored[i]) m++; }
        std::vector<int> reducedToFull;
        for (int i = 0; i < n; i++) {
            if (explored[i]) {
                reducedToFull.push_back(i);
            }
        }

        Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(n, n);
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                int ii = reducedToFull[i];
                int jj = reducedToFull[j];
                ret(ii, jj) = M(i, j);
            }
        }
        return ret;
    }

    // int BoumanCluster::majorClass() {
    //     int cnt[10];
    //     for (int i = 0; i < 10; i++) {
    //         cnt[i] = 0;
    //     }
    //     for (int i = 0; i < cls.size(); i++) {
    //         cnt[cls[i]]++;
    //     }

    //     int max = -1;
    //     int maxIndex = 0;
    //     for (int i = 0; i < 10; i++) {
    //         if (cnt[i] > max) {
    //             max = cnt[i];
    //             maxIndex = i;
    //         }
    //     }
    //     return maxIndex;

    // }
    
    int BoumanCluster::numClasses() {
        return means.size();
        // int maxCls = -1;
        // for (int i = 0; i < cls.size(); i++) {
        //     if (maxCls < cls[i]) {
        //         maxCls = cls[i];
        //     }
        // }
        // return maxCls + 1;
    }

    Sample* BoumanCluster::bestSampleInClass(int ci) {
        Sample* ret = NULL;
        for (int i = 0; i < cls.size(); i++) {
            if (cls[i] != ci) {
                continue;
            }
            if (ret == NULL || (ret->value) > (goodsamples[i]->value) ) {
                ret = goodsamples[i];
            }
        }
        return ret;
    }

    // int BoumanCluster::getClass(int index) {
    //     return cls[index];
    // }

    int BoumanCluster::nChildProc() {
        return cmd()->nChildProc();
    }
    
    Database* BoumanCluster::db() {
        return cmd()->database();
    }


} // namespace mpisolver2
