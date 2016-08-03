/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

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

#define COUT if (false) cout

namespace solver {
    double BoumanCluster::g_clusterMinDist = 0.3;
    
    BoumanCluster::BoumanCluster(std::vector<Sample*> _samples)
        : samples(_samples)
    {
    }
    
    BoumanCluster::~BoumanCluster() {
    }

    void BoumanCluster::cluster() {
        nClasses = 0;
        cls.clear();

        // cout << "run().." << endl;
        run();
        // cout << "cluster()..OK" << endl;
        LOG_INFO << FUNCTION_NAME() << " OK";
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
        
        int DIM = samples[0]->params.size();


        SigSet S;
        memset(&S, 0, sizeof(SigSet));
        /* Initialize SigSet data structure */
        I_InitSigSet (&S);
        I_SigSetNBands (&S, DIM);
        I_SetSigTitle (&S, (char*)"BoumanCluster signature set");

        /* Allocate memory for cluster signatures */
        // LOG_INFO << "Allocate memory for cluster signatures";
        for (int k = 0; k < NCLASSES; k++) {
            ClassSig* Sig = I_NewClassSig(&S);
            I_SetClassTitle (Sig, (char*)"test class signature");
            for(int i = 0; i < NINITSUBCLASSES; i++) {
                I_NewSubSig (&S, Sig);
            }
        }

        int NSAMPLES = samples.size();
        /* Read data for each class */
        // LOG_INFO << "Read data for each class";
        for (int k = 0; k < NCLASSES; k++) {
            struct ClassSig* Sig = &(S.ClassSig[k]);
            I_AllocClassData (&S, Sig, NSAMPLES);

            /* Read Data */
            for(int i = 0; i < Sig->ClassData.npixels; i++) {
                const Eigen::VectorXd& gene = samples[i]->params;
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
        // LOG_INFO << "Compute the average variance over all classes";
        double Rmin = 0;
        for(int k = 0; k < NCLASSES; k++) {
            struct ClassSig* Sig = &(S.ClassSig[k]);
            Rmin += AverageVariance(Sig, DIM);
        }
        Rmin = Rmin/(COVAR_DYNAMIC_RANGE * NCLASSES);

        // FILE* fp = fopen("sigset.txt", "w+");
        // I_WriteSigSet(fp, &S);
        // fclose(fp);
        // LOG(INFO) << "try to save sigset...OK";
        // cout << "try to save sigset OK!!!!!!!!";


        // LOG_INFO << "subclustering..";
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

        // LOG_INFO << "read result";
        /* read the result and parse into class members */
        int nsubclasses = 0;
        {
            struct ClassSig* Cp = &(S.ClassSig[0]);
            nsubclasses = Cp->nsubclasses;
            for (int j = 0; j < Cp->nsubclasses; j++) {
                struct SubSig* Sp = &Cp->SubSig[j];
            }

        }
        LOG_INFO << "Result = " << nsubclasses;
        this->nClasses = nsubclasses;

        ////////////////////////////////////////////////////////////
        // To classify each sample, we need to flatten the structure(SplitClasses.c)
        // LOG_INFO << "flatten the structure(SplitClasses.c)";
        struct SigSet& Sin = S;
        struct SigSet Sout;
        /* Initialize SigSet data structure */
        I_InitSigSet (&Sout);
        COUT << "I_InitSigSet" << endl;
        I_SigSetNBands (&Sout, Sin.nbands);
        COUT << "I_SigSetNBands" << endl;
        I_SetSigTitle (&Sout, (char*)"signature set for unsupervised clustering");
        COUT << "I_SetSigTitle" << endl;

        /* Copy each subcluster (subsignature) from input to cluster (class signature) of output */
        // LOG_INFO << "Copy each subcluster";

        clsMeans.clear();
        for(int k = 0; k < Sin.nclasses; k++) {
            COUT << "loop k = " << k << endl;
            for(int l = 0; l < (Sin.ClassSig[k].nsubclasses); l++) {
                COUT << "loop l = " << l << endl;
                struct ClassSig* Sig = I_NewClassSig(&Sout);
                COUT << "I_NewClassSig" << endl;
                I_SetClassTitle (Sig, (char*)"Single Model Class");
                COUT << "I_SetClassTitle" << endl;
                I_NewSubSig (&Sout, Sig);
                COUT << "I_NewSubSig" << endl;
                Sig->SubSig[0].pi = 1.0;
                COUT << "start loop for setting means" << endl;
                Eigen::VectorXd mean(DIM);
                for(int i = 0; i < Sin.nbands; i++) {
                    Sig->SubSig[0].means[i] = Sin.ClassSig[k].SubSig[l].means[i];
                    mean(i) = Sin.ClassSig[k].SubSig[l].means[i];
                }
                clsMeans.push_back(mean);
                COUT << "start loop for setting covs" << endl;
                for(int i = 0; i < Sin.nbands; i++) {
                    for(int j = 0; j < Sin.nbands; j++) {
                        Sig->SubSig[0].R[i][j] = Sin.ClassSig[k].SubSig[l].R[i][j];
                    }
                }
            }
        }
        LOG_INFO << "Flatten OK";
        
        /* Initialize constants for Log likelihood calculations */
        ClassLogLikelihood_init(&Sout);
        COUT << "ClassLogLikelihood_init" << endl;
        /* Compute Log likelihood for each class*/
        // LOG_INFO << "Compute Log likelihood for each class";
        // double* ll = G_alloc_vector(S.nclasses + 1);
        double* ll = G_alloc_vector(Sout.nclasses + 1);
        COUT << "G_alloc_vector: Sout.nclasses = " << Sout.nclasses << endl;

        double* data = new double[DIM + 1];
        COUT << "DIM = " << DIM << endl;
        for(int i = 0; i < samples.size(); i++) {
            COUT << "testing loop i = " << i << endl;
            Sample* gs = samples[i];
            const Eigen::VectorXd& gene = gs->params;
            for (int j = 0; j < DIM; j++) {
                data[j] = gene(j);
            }
            ClassLogLikelihood(data,ll,&Sout);
            COUT << "ClassLogLikelihood: Sout.nclasses= " << Sout.nclasses << endl;
            double maxval = ll[0];
            COUT << "maxval = " << maxval << endl;
            int maxindex = 0;
            for(int j = 0; j < Sout.nclasses; j++) {
                COUT << "Sout.class loop j = " << j << endl;
                if( ll[j] > maxval ) {
                    maxval = ll[j];
                    maxindex = j;
                }
            }
            COUT << "now push_back the index..." << endl;
            // gs->debugClusterId = maxindex;
            cls.push_back(maxindex);
            // LOG_INFO << "Sample " << i << " "
            //          << ": class = " << maxindex;


        }
        delete[] data;

        // I_DeallocSigSet(&S); 

        LOG_INFO << FUNCTION_NAME() << " OK";
        collapseByMeans();
    }

    void BoumanCluster::collapseByMeans() {
        if (clsMeans.size() < 2) {
            return;
        }
        Eigen::VectorXd lhs = clsMeans[0];
        Eigen::VectorXd rhs = clsMeans[1];
        LOG_INFO << FUNCTION_NAME();
        double dist = (lhs - rhs).norm();
        LOG_INFO << "Compare dist " << dist << " and minDist " << g_clusterMinDist
                 << " mean0 = " << IO(lhs) << " mean1 = " << IO(rhs);
        if (dist < g_clusterMinDist) {
            LOG_INFO << "Collapse into one cluster due to their distance !!!##!!";
            nClasses = 1;
            for (int i = 0; i < cls.size(); i++) {
                cls[i] = 0;
            }
        }
    }

    std::vector<Sample*> BoumanCluster::getCluster(int index) {
        std::vector<Sample*> ret;
        for (int i = 0; i < samples.size(); i++) {
            if (getClass(i) == index) {
                ret.push_back(samples[i]);
            }
        }
        return ret;
    }

    
} // namespace solver



