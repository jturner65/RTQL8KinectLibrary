#include "rcluster.h"

#include <algorithm>
#include <fstream>
#include "utils/UtilsMath.h"
#include "common/app_cppcommon.h"

#include "controller/SimPack.h"
#include "controller/controller"

#include "commandcenter.h"
#include "database.h"

namespace mpisolver2 {
    Rcluster::Rcluster(CommandCenter* _cmd)
        : MEMBER_INIT(cmd, _cmd)
    {
    }
    
    Rcluster::~Rcluster() {
    }

    void Rcluster::cluster() {
        goodsamples.clear();
        cls.clear();
        means.clear();
        covs.clear();

        // select();
        cout << "selectByRatio().." << endl;
        selectByRatio();
        cout << "write().." << endl;
        write();
        cout << "run().." << endl;
        run();
        cout << "read().." << endl;
        read();
        cout << "cluster()..OK" << endl;
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }
    void Rcluster::select() {
        // Select 33% of sampels
        db()->sortByPrevValue();
        int n = db()->numSamples() / 3;
        n = std::min(n, 16);
        
        for (int i = 0; i < n; i++) {
            Sample* s = db()->sample(i);
            goodsamples.push_back(s);
        }
        LOG(INFO) << FUNCTION_NAME() << " : the number of selected samples = " << n;
    }

    void Rcluster::selectByRatio() {
        const double ratio = 2.0;
        
        db()->sort();
        double minValue = db()->sample(0)->prevValue;
        double upperBound = ratio * minValue;
        LOG(INFO) << FUNCTION_NAME()
                  << " minValue = " << minValue
                  << " upperBound = " << upperBound;

        for (int i = 0; i < db()->numSamples(); i++) {
            Sample* s = db()->sample(i);
            if (s->prevValue > upperBound) {
                break;
            }
            goodsamples.push_back(s);
        }
        
        LOG(INFO) << FUNCTION_NAME()
                  << " : the number of selected samples = " << goodsamples.size();
    }
    

    void Rcluster::write() {
        // Write to the database
        std::ofstream fout("rclust_data.csv");

        std::vector<bool> explored = db()->parameterExplored();
        int n = cmd()->dim();
        int m = 0;
        for (int i = 0; i < explored.size(); i++) {
            if (explored[i]) m++;
            cout << "dim " << i << " explored = " << explored[i] << endl;
        }
        cout << "Original dimension: " << n << " Explored dimension: " << m << endl;

        for (int i = 0; i < m; i++) {
        	char ch = 'a' + i;
        	if (i != 0) fout << ", ";
        	fout << ch;
        }
        fout << endl;

        for (int i = 0; i < goodsamples.size(); i++) {
            // const Eigen::VectorXd& gene = goodsamples[i]->params;
            const Eigen::VectorXd& gene = goodsamples[i]->reducedParam( explored );
            
            for (int j = 0; j < m; j++) {
            	if (j != 0) fout << ", ";
                // if (fabs(gene(j)) < 0.0000001) {
                //     fout << rtql8::utils::random(-1, 1);
                // } else {
                fout << gene(j);
                // }
            }
            fout << endl;
//            fout << gene(0);
//            fout << ", " << gene(1);
//            fout << endl;
        }
        fout.close();
        LOG(INFO) << FUNCTION_NAME() << " OK";
    }
    
    void Rcluster::run() {
        execute("rm rclust_result.csv");
        for (int i = 1; i <= 10; i++) {
            std::string cmd =
                (boost::format(
                    "rm rclust_result_mean_%d.csv") % i
                    ).str();
            execute(cmd.c_str());
        }
        for (int i = 1; i <= 10; i++) {
            std::string cmd =
                (boost::format(
                    "rm rclust_result_cov_%d.csv") % i
                    ).str();
            execute(cmd.c_str());
        }
        execute("R CMD BATCH rclust_script.R");
    }
    
    void Rcluster::read() {
        std::ifstream fin("rclust_result.csv");
        std::string temp;
        fin >> temp;
        for (int i = 0; i < goodsamples.size(); i++) {
            int c;
            fin >> c;
            c--; // Change from 1-based index to 0-based index

            // c = 0;
            cls.push_back(c);
            cout << "class " << i << " = " << c << endl;
        }

        int NC = numClasses();
        for (int i = 1; i <= NC; i++) {
            Eigen::VectorXd mean = readMean(i);
            Eigen::MatrixXd cov = readCov(i);
            means.push_back(mean);
            covs.push_back(cov);
        }
        LOG(INFO) << "numClasses = " << numClasses() << ", majorClass = " << majorClass();
    }

    Eigen::VectorXd Rcluster::readMean(int cls) {
        std::vector<bool> explored = db()->parameterExplored();
        int n = cmd()->dim();
        int m = 0;
        for (int i = 0; i < explored.size(); i++) { if (explored[i]) m++; }


        std::string filename = (boost::format(
            "rclust_result_mean_%d.csv") % cls
            ).str();
        std::ifstream fin(filename.c_str());
        std::string temp;
        fin >> temp; // Ignore the first 

        Eigen::VectorXd reduced( m );
        for (int i = 0; i < m; i++) {
            double v;
            fin >> v;
            reduced(i) = v;
        }

        Eigen::VectorXd mean = Eigen::VectorXd::Zero(n);
        int ptr = 0;
        for (int i = 0; i < n; i++) {
            if (!explored[i]) continue;
            mean(i) = reduced(ptr++);
        }
        CHECK_EQ(ptr, m);
        
        LOG(INFO) << FUNCTION_NAME() << " OK : cls = " << cls << " mean = " << IO(mean);
        return mean;
    }
    

    Eigen::MatrixXd Rcluster::readCov(int cls) {
        std::vector<bool> explored = db()->parameterExplored();
        int n = cmd()->dim();
        int m = 0;
        for (int i = 0; i < explored.size(); i++) { if (explored[i]) m++; }
        std::vector<int> reducedToFull;
        for (int i = 0; i < n; i++) {
            if (explored[i]) {
                reducedToFull.push_back(i);
            }
        }
        // for (int i = 0; i < reducedToFull.size(); i++) {
        //     cout << "map " << i << " --> " << reducedToFull[i] << endl;
        // }


        std::string filename = (boost::format(
            "rclust_result_cov_%d.csv") % cls
            ).str();
        std::ifstream fin(filename.c_str());
        std::string temp;
        fin >> temp; // Ignore the first 

        // Eigen::MatrixXd cov(n, n);
        Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(n, n);
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                if (j != 0) {
                    char comma;
                    fin >> comma;
                }
                double v;
                fin >> v;
                int ii = reducedToFull[i];
                int jj = reducedToFull[j];
                cov(ii, jj) = v;
            }
        }

        LOG(INFO) << FUNCTION_NAME() << " OK : cls = " << cls << " cov = " << endl << cov;
        return cov;
        
    }

    int Rcluster::majorClass() {
        int cnt[10];
        for (int i = 0; i < 10; i++) {
            cnt[i] = 0;
        }
        for (int i = 0; i < cls.size(); i++) {
            cnt[cls[i]]++;
        }

        int max = -1;
        int maxIndex = 0;
        for (int i = 0; i < 10; i++) {
            if (cnt[i] > max) {
                max = cnt[i];
                maxIndex = i;
            }
        }
        return maxIndex;

    }
    
    int Rcluster::numClasses() {
        int maxCls = -1;
        for (int i = 0; i < cls.size(); i++) {
            if (maxCls < cls[i]) {
                maxCls = cls[i];
            }
        }
        return maxCls + 1;
    }
    
    int Rcluster::getClass(int index) {
        return cls[index];
    }


    int Rcluster::nChildProc() {
        return cmd()->nChildProc();
    }
    controller::SimPack* Rcluster::simpack() {
        return cmd()->simpack();
    }
    
    Database* Rcluster::db() {
        return cmd()->database();
    }

    bool Rcluster::execute(const char* const cmd) {
        FILE* pipe = popen(cmd, "r");
        if (!pipe) {
            return false;
        }
        
        char buffer[128];
        while(!feof(pipe)) {
            if(fgets(buffer, 128, pipe) != NULL) {
                cout << buffer << std::flush;
            }
        }
        pclose(pipe);
        return true;
    }
} // namespace mpisolver2
