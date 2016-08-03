#include "Rclustering.h"

#include <fstream>
#include "common/app_cppcommon.h"
#include "MPICMASearch.h"

namespace mpisolver {
    Rclustering::Rclustering(MPICMASearch* _cma)
        : MEMBER_INIT(cma, _cma)
    {
    }

    Rclustering::~Rclustering() {
    }

    void Rclustering::cluster() {
        cls.clear();
        write();
        run();
        read();
        LOG(INFO) << FUNCTION_NAME() << " OK";
        // exit(0);
    }
        
    void Rclustering::write() {
        std::ofstream fout("rclust_data.csv");
        fout << "x, y" << endl;
        for (int i = 0; i < cma()->numParents(); i++) {
            Eigen::VectorXd gene = cma()->parent(i);
            double value = cma()->parentValue(i);
            fout << gene(0);
            fout << ", " << gene(1);
            fout << endl;
        }
        fout.close();
        LOG(INFO) << FUNCTION_NAME() << " OK";

    }
    
    void Rclustering::run() {
        execute("rm rclust_result.csv");
        execute("R CMD BATCH rclust_script.R");
    }
    
    void Rclustering::read() {
        std::ifstream fin("rclust_result.csv");
        std::string temp;
        fin >> temp;
        int maxClass = 0;
        for (int i = 0; i < cma()->numParents(); i++) {
            int c;
            fin >> c;
            c--; // Change from 1-based index to 0-based index

            // c = 0;
            cls.push_back(c);
            cout << "class " << i << " = " << c << endl;
            if (maxClass < c) {
                maxClass = c;
            }
        }
        cout << "# classes = " << maxClass + 1 << ", major = " << majorGroup() << endl;
    }

    int Rclustering::majorGroup() {
        // return 0;
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

    int Rclustering::numGroups() {
        // return 1;
        int maxCls = -1;
        for (int i = 0; i < cls.size(); i++) {
            if (maxCls < cls[i]) {
                maxCls = cls[i];
            }
        }
        return maxCls + 1;

    }

    bool Rclustering::execute(const char* const cmd) {
        FILE* pipe = popen(cmd, "r");
        if (!pipe) {
            return false;
        }
        
        char buffer[128];
        // std::string result = "";
        while(!feof(pipe)) {
            if(fgets(buffer, 128, pipe) != NULL) {
                cout << buffer << std::flush;
    		// result += buffer;
            }
        }
        pclose(pipe);
        return true;
    }


} // namespace mpisolver
