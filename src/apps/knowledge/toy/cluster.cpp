/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "cluster.h"

#include <fstream>
#include "common/app_cppcommon.h"

extern "C" {
#include "cluster/clust_defs.h"
#include "cluster/alloc_util.h"
#include "cluster/clust_io.h"
#include "cluster/clust_util.h"
#include "cluster/subcluster.h"
#include "cluster/classify_util.h"
}


namespace toy {

////////////////////////////////////////////////////////////
// class Cluster implementation
    Cluster::Cluster() {
    }

    Cluster::Cluster(std::vector<toy::Sample>& _samples)
        : samples(_samples)
    {
    }
    
    Cluster::~Cluster() {
    }

    void Cluster::test() {
        cout << "Hello, Cluster" << endl;
        // BOOST_FOREACH(const Sample& s, samples) {
        //     cout << s << endl;
        // }
        // cout << "samples.size() = " << samples.size() << endl;

        Set samples_top  = samples;
        Set samples_mid;
        split_w_task(samples_top, 0.3, 0.7, samples_mid);
        // cout << "samples_top.size() = " << samples_top.size() << endl;
        // cout << "samples_mid.size() = " << samples_mid.size() << endl;

        Set samples_bot;
        split_w_task(samples_top, 0.0, 0.3, samples_bot);

        cout << "samples_top.size() = " << samples_top.size() << endl;
        cout << "samples_mid.size() = " << samples_mid.size() << endl;
        cout << "samples_bot.size() = " << samples_bot.size() << endl;



        Set top0 = samples_top;
        Set top1;
        split_w_cluster(top0, top1);

        Set bot0 = samples_bot;
        Set bot1;
        split_w_cluster(bot0, bot1);

        writeFileWithGroup(cout, top0, "top0");
        writeFileWithGroup(cout, top1, "top1");
        writeFileWithGroup(cout, samples_mid, "mid");
        writeFileWithGroup(cout, bot0, "bot0");
        writeFileWithGroup(cout, bot1, "bot1");
        

        std::ofstream fout("lean_cluster.csv");
        int DIM = 2;
        for (int i = 0; i < DIM; i++) {
            char c = 'a' + i;
            fout << c << ", ";
        }
        fout << "task, error, group" << endl;

        writeFileWithGroup(fout, top0, "top0");
        writeFileWithGroup(fout, top1, "top1");
        writeFileWithGroup(fout, samples_mid, "mid");
        writeFileWithGroup(fout, bot0, "bot0");
        writeFileWithGroup(fout, bot1, "bot1");


        clusters.push_back( merge(top0, samples_mid, bot0) );
        clusters.push_back( merge(top1, samples_mid, bot0) );


        cout << "CLUSTER OK" << endl;
        // exit(0);
    }

    void Cluster::split_w_task(Set& input, double task_lo, double task_hi, Set& output) {
        Set input_new;
        output.clear();
        BOOST_FOREACH(const Sample& s, input) {
            if (s.task < task_lo || task_hi < s.task) {
                input_new.push_back(s);
            } else {
                output.push_back(s);
            }

        }
        // cout << "output.size() = " << output.size() << endl;
        // cout << "input_new.size() = " << input_new.size() << endl;
        input = input_new;
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

    void Cluster::split_w_cluster(Set& input, Set& output) {
        int NCLASSES = 1; // The number of class before doing clustering: we have only one
        int NINITSUBCLASSES = 2; // The number of initial subclasses (results): maybe adjusted

        clusterMessageVerboseLevel = 2;
        

        int DIM = input[0].params.size();


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

        int NSAMPLES = input.size();
        /* Read data for each class */
        for (int k = 0; k < NCLASSES; k++) {
            struct ClassSig* Sig = &(S.ClassSig[k]);
            I_AllocClassData (&S, Sig, NSAMPLES);


            /* Read Data */
            for(int i = 0; i < Sig->ClassData.npixels; i++) {
                const Eigen::VectorXd& gene = input[i].params;
                cout << i << " : " << gene.transpose() << endl;
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
                means.push_back(m);

                Eigen::MatrixXd C( DIM, DIM );
                for (int q = 0; q < DIM; q++) {
                    for (int w = 0; w < DIM; w++) {
                        C(q, w) = Sp->R[q][w];
                    }
                }
                covs.push_back(C);
                
                LOG_INFO << "Class " << j;
                LOG_INFO << "Mean = " << IO(m);
                LOG_INFO << "Cov = " << endl << C;
            }
        }

        LOG_INFO << "Result = " << nsubclasses;

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
        LOG_INFO << "Flatten OK";
        
        /* Initialize constants for Log likelihood calculations */
        ClassLogLikelihood_init(&Sout);

        /* Compute Log likelihood for each class*/
        Set set_a;
        Set set_b;
        
        double* ll = G_alloc_vector(S.nclasses);
        double* data = new double[DIM + 1];
        for(int i = 0; i < input.size(); i++) {
            const Eigen::VectorXd& gene = input[i].params;
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

            // LOG_INFO << "dddd" << endl;
            if (maxindex == 0) set_a.push_back(input[i]);
            else if (maxindex == 1) set_b.push_back(input[i]);
            else {
                cerr << "invalid cluster: " << maxindex << endl;
                exit(4);
            }

            LOG_INFO << "Sample " << i << " " << gene.transpose()
                     << " : class = " << maxindex << endl;


        }
        delete[] data;

        I_DeallocSigSet(&S); 

        input = set_a;
        output = set_b;
        
        
    }

    void Cluster::writeFileWithGroup(std::ostream& fout, Set& input,
                                     std::string group) {
        BOOST_FOREACH(const Sample& s, input) {
            for (int j = 0; j < s.params.size(); j++) {
                fout << boost::format("% 6lf,") % s.params(j);
            }
            fout << boost::format("% 6lf,% 6lf") % s.task % s.error;
            fout << "," << group << endl;
        }
    }


    Cluster::Set Cluster::merge(Set& lhs, Set& rhs) {
        Set output;
        BOOST_FOREACH(const Sample& s, lhs) {
            output.push_back(s);
        }
        BOOST_FOREACH(const Sample& s, rhs) {
            output.push_back(s);
        }
        return output;
    }
    
    Cluster::Set Cluster::merge(Set& c0, Set& c1, Set& c2) {
        Set output;
        BOOST_FOREACH(const Sample& s, c0) {
            output.push_back(s);
        }
        BOOST_FOREACH(const Sample& s, c1) {
            output.push_back(s);
        }
        BOOST_FOREACH(const Sample& s, c2) {
            output.push_back(s);
        }
        return output;
    }

// class Cluster ends
////////////////////////////////////////////////////////////
    

    
} // namespace toy



