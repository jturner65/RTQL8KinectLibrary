#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <Eigen/Dense>
#include "EigenQP.h"
#include "utils/EigenHelper.h"
#include "utils/UtilsMath.h"
#include "prob.h"
#include "cluster.h"
using namespace std;

void fit(toy::Model* m, const std::vector<toy::Sample>& samples) {
    const int DIM = m->dim;
    double sum_w   = 0.0;
    double sum_wy  = 0.0;
    double sum_wy2 = 0.0;

    Eigen::VectorXd sum_wp  = Eigen::VectorXd::Zero(DIM);
    Eigen::VectorXd sum_wpy = Eigen::VectorXd::Zero(DIM);
    Eigen::VectorXd sum_wp2 = Eigen::VectorXd::Zero(DIM);

    Eigen::VectorXd reg = Eigen::VectorXd::Zero(DIM * 2);
    Eigen::VectorXd reg_alpha = reg.head(DIM);
    Eigen::VectorXd reg_beta  = reg.tail(DIM);
    double sum_value = 0.0;

    BOOST_FOREACH(const toy::Sample& s, samples) {
        // double w = 1.0 / (0.0001 + s.error);
        double w = 1.0 / ( std::max(s.error, 0.01) );

        sum_w   += w;
        sum_wy  += w * s.task;
        sum_wy2 += w * s.task * s.task;

        for (int i = 0; i < DIM; i++) {
            sum_wp(i)  += w * s.params(i);
            sum_wpy(i) += w * s.params(i) * s.task;
            sum_wp2(i) += w * s.params(i) * s.params(i);
        }

        sum_value += w * ( (s.params - (reg_alpha + s.task * reg_beta)).squaredNorm() );

    }

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(DIM * 2, DIM * 2);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(DIM * 2);
    double C = 0.0;
    for (int i = 0; i < DIM; i++) {
        int j = i + DIM;

        D(i, i) = sum_w;
        D(i, j) = sum_wy;
        D(j, i) = sum_wy;
        D(j, j) = sum_wy2;

        B(i) = sum_wp(i);
        B(j) = sum_wpy(i);

        C += sum_wp2(i);
    }

    cout << "sum_value = " << sum_value << endl;
    double mat_value = reg.dot(D * reg) - 2.0 * B.dot(reg) + C;
    cout << "mat_value = " << mat_value << endl;

    double ratio = 0.000001;
    for (int i = 0; i <= 10000; i++) {
        Eigen::Vector4d grad = 2.0 * D * reg - 2.0 * B;

        if ( (ratio * grad).norm() < 1e-6) {
            break;
        }
        reg -= ratio * grad;
        // double mat_value = alpha.dot(D * alpha) - 2.0 * B.dot(alpha) + C;
        // if (i % 100 == 0 || terminate) {
        //     cout << "grad = " << grad.transpose() << endl;
        //     cout << i << " : " << alpha.transpose() << " --> " << boost::format("%.8lf") % mat_value << endl;
        // }
    }
    exit(0);

    double optimized = reg.dot(D * reg) - 2.0 * B.dot(reg) + C;
    cout << "optimized = " << optimized << endl;

    m->alpha = reg.head(DIM);
    m->beta  = reg.tail(DIM);
    m->C *= 0.9;
}


void fit_bezier(toy::ModelBezier* m, const std::vector<toy::Sample>& samples) {
    const int DIM = m->dim;

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(DIM * 4, DIM * 4);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(DIM * 4);
    double C = 0.0;

    // Eigen::VectorXd reg = Eigen::VectorXd::Random(DIM * 4);
    // Eigen::VectorXd reg = Eigen::VectorXd::Zero(DIM * 4);
    // Eigen::VectorXd reg0 = reg.segment(DIM * 0, DIM);
    // Eigen::VectorXd reg1 = reg.segment(DIM * 1, DIM);
    // Eigen::VectorXd reg2 = reg.segment(DIM * 2, DIM);
    // Eigen::VectorXd reg3 = reg.segment(DIM * 3, DIM);

    Eigen::VectorXd reg0 = m->alpha;
    Eigen::VectorXd reg1 = m->beta;
    Eigen::VectorXd reg2 = m->gamma;
    Eigen::VectorXd reg3 = m->delta;
    Eigen::VectorXd reg = Eigen::VectorXd::Zero(DIM * 4);
    reg.segment(DIM * 0, DIM) = reg0;
    reg.segment(DIM * 1, DIM) = reg1;
    reg.segment(DIM * 2, DIM) = reg2;
    reg.segment(DIM * 3, DIM) = reg3;
    
    
    double sum_value = 0.0;

    BOOST_FOREACH(const toy::Sample& s, samples) {
        // double w = 1.0 / (0.0001 + s.error);
        // double w = 1.0 / (0.0001 + max(0.01, s.error));
        double w = 1.0;
        double t = s.task;

        double a = (1 - t) * (1 - t) * (1 - t);
        double b = 3.0 * (1 - t) * (1 - t) * t;
        double c = 3.0 * (1 - t) * t * t;
        double d = t * t * t;

        // cout << "t = " << t << endl;
        // cout << "a,b,c,d, = " << a << ", " << b << ", " << c << ", " << d << endl;

        Eigen::Vector4d abcd;
        abcd << a, b, c, d;
        Eigen::Matrix4d coef;
        coef <<
            a * a, a * b, a * c, a * d,
            b * a, b * b, b * c, b * d,
            c * a, c * b, c * c, c * d,
            d * a, d * b, d * c, d * d;
        // cout << coef << endl;

        for (int k = 0; k < DIM; k++) {
            for (int i = 0; i < coef.rows(); i++) {
                int y = DIM * i + k;
                for (int j = 0; j < coef.cols(); j++) {
                    int x = DIM * j + k;
                    D(y, x) += w * coef(i, j);
                }
                B(y) += w * abcd(i) * s.params(k); 
            }
            C += w * s.params(k) * s.params(k);
        }
        // cout << D << endl;
        // cout << B << endl;
        // cout << C << endl;
        // exit(0);
        sum_value += w * ( (s.params - (a * reg0 + b * reg1 + c * reg2 + d * reg3)).squaredNorm() );

    }


    cout << "sum_value = " << sum_value << endl;
    double mat_value = reg.dot(D * reg) - 2.0 * B.dot(reg) + C;
    cout << "mat_value = " << mat_value << endl;
    

    {
        const int n = DIM * 4;
        Eigen::MatrixXd G   = 2.0 * D;
        Eigen::VectorXd g0  = -2.0 * B;

        Eigen::MatrixXd CE  = Eigen::MatrixXd::Zero(n, 0);
        Eigen::VectorXd ce0 = Eigen::VectorXd::Zero(0);

        Eigen::MatrixXd CI  = Eigen::MatrixXd::Zero(n, 0);
        Eigen::VectorXd ci0 = Eigen::VectorXd::Zero(0);

        Eigen::VectorXd x   = Eigen::VectorXd::Zero(n);

        double value = QP::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
        cout << "value = " << value << endl;
        cout << "value + C = " << value + C << endl;
        reg = x;
    }

    // double ratio = 0.0000001;
    // for (int i = 0; i <= 1000000; i++) {
    //     Eigen::Vector4d grad = 2.0 * D * reg - 2.0 * B;
    //     // if ( (ratio * grad).norm() < 1e-10) {
    //     //     break;
    //     // }
    //     reg -= ratio * grad;
    //     // double mat_value = alpha.dot(D * alpha) - 2.0 * B.dot(alpha) + C;
    //     // if (i % 100 == 0 || terminate) {
    //     //     cout << "grad = " << grad.transpose() << endl;
    //     //     cout << i << " : " << alpha.transpose() << " --> " << boost::format("%.8lf") % mat_value << endl;
    //     // }
    // }


    double optimized = reg.dot(D * reg) - 2.0 * B.dot(reg) + C;
    cout << "optimized = " << optimized << endl;
    cout << "reg = " << reg.transpose() << endl;
    m->alpha = reg.segment(DIM * 0, DIM);
    m->beta  = reg.segment(DIM * 1, DIM);
    m->gamma = reg.segment(DIM * 2, DIM);
    m->delta = reg.segment(DIM * 3, DIM);
    m->C *= 0.9;

    // m->alpha = (Eigen::VectorXd(2) <<  0.82,  0.62).finished();
    // m->beta  = (Eigen::VectorXd(2) << -0.30,  1.00).finished();
    // m->gamma = (Eigen::VectorXd(2) << -0.00,  1.00).finished();
    // m->delta = (Eigen::VectorXd(2) << -0.50, -0.50).finished();
                            
    // int index = 0;
    // double value = 0.0;
    // BOOST_FOREACH(const toy::Sample& s, samples) {
    //     double w = 1.0;
    //     double t = s.task;

    //     double a = (1 - t) * (1 - t) * (1 - t);
    //     double b = 3.0 * (1 - t) * (1 - t) * t;
    //     double c = 3.0 * (1 - t) * t * t;
    //     double d = t * t * t;
    //     Eigen::VectorXd orig = s.params;
    //     Eigen::VectorXd curr = (a * m->alpha + b * m->beta + c * m->gamma + d * m->delta);
    //     double temp = w * ( (orig - curr).squaredNorm() );
    //     value += temp;
    //     cout << index << " : " << temp << " :::: " << orig.transpose() << " <--> " << curr.transpose() << endl;
    // }
    // cout << "sum.optimized = " << value << endl;

}

void moo_sort(std::vector<toy::Sample>& samples) {
    const int MAX_SIZE = 1000;
    bool picked[MAX_SIZE];
    for (int i = 0; i < samples.size(); i++) {
        picked[i] = false;
    }
    std::vector<toy::Sample> sorted;

    int loop_counter = 0;
    while( sorted.size() < samples.size() ) {
        // Step 1. pick the best one
        {
            int index = -1;
            double min_error = 99999999.0;
            for (int i = 0; i < samples.size(); i++) {
                if (picked[i]) continue;
                if (min_error > samples[i].error) {
                    min_error = samples[i].error;
                    index = i;
                }
            }
            if (index == -1) { cerr << "cannot pick the best one" << endl; exit(3); }
            picked[index] = true;
            sorted.push_back(samples[index]);
            cout << "best one: " << index << " : " << min_error << "("
                 << samples[index] << ")" << endl;
        }

        if (sorted.size() >= samples.size()) {
            break;
        }
        
        // Step 2. pick the max distance in task space
        {
            int index = -1;
            double max_dist = -1.0;
            for (int i = 0; i < samples.size(); i++) {
                if (picked[i]) continue;
                double dist = 999999.0;
                for (int j = 0; j < samples.size(); j++) {
                    if (picked[j] == false) continue;
                    // double dist_now = (samples[i].params - samples[j].params).norm();
                    double dist_now = fabs(samples[i].task - samples[j].task);
                    if (dist > dist_now) {
                        dist = dist_now;
                    }
                }
                
                if (max_dist < dist) {
                    max_dist = dist;
                    index = i;
                }
            }
            if (index == -1) { cerr << "cannot pick the most sparse" << endl; exit(3); }
            picked[index] = true;
            sorted.push_back(samples[index]);
            cout << "sparse one: " << index << " : " << max_dist << "("
                 << samples[index] << ")" << endl;
        }

        //
        loop_counter++;
        if (loop_counter > 1000) {
            cerr << "something wrong: infinite loop" << endl;
            exit(3);
        }

    }
    samples = sorted;
}

void loadSample(std::vector<toy::Sample>& samples) {
    std::ifstream fin("test.csv");
    if (!fin.is_open()) {
        cerr << "test.csv not found" << endl;
        exit(2);
    }

    const int M = 6;
    const int DIM = 2;
    for (int i = 0; i < M; i++) {
        std::string temp;
        fin >> temp;
        cout << "column " <<i << " = " << temp << endl;
    }
    while(1) {
        toy::Sample s;
        s.params = Eigen::VectorXd::Zero(DIM);
        
        for (int i = 0; i < M; i++) {
            std::string temp;
            switch(i) {
            case 0: fin >> temp; break;
            case 1: fin >> temp; break;
            case 4: fin >> s.task >> temp; break;
            case 5: fin >> s.error; break;
            default:
                fin >> s.params(i - 2) >> temp;
                // cout << s.params(i - 2) << endl;
            }
            // cout << "i " << i << endl;
        }
        if (fin.fail()) {
            break;
        }
        // cout << "Sample = " << s << endl;
        samples.push_back(s);
    }
    
}

void saveSample(std::vector<toy::Sample>& samples) {
    int DIM = samples[0].params.size();

    std::ofstream fout("output.csv");
    for (int i = 0; i < DIM; i++) {
        char c = 'a' + i;
        fout << c << ", ";
    }
    fout << "task, error" << endl;
    for (int i = 0; i < samples.size(); i++) {
        toy::Sample& s = samples[i];
        for (int j = 0; j < DIM; j++) {
            fout << boost::format("% 6lf, ") % s.params(j);
        }
        fout << boost::format("% 6lf, % 6lf") % s.task % s.error << endl;
    }
    fout.close();
}

void exploreSamples(toy::Prob* prob) {
    const int NSAMPLES = 3000;
    std::vector<toy::Sample> samples;

    for (int i = 0; i < NSAMPLES; i++) {
        toy::Sample s;
        s.params = Eigen::VectorXd::Random(prob->dim);
        prob->query(&s);
        samples.push_back(s);
    }
    saveSample(samples);
    cout << "Done with the exploring!!! Goodbye" << endl;
}

int main() {

    std::vector<toy::Sample> test_population;
    loadSample(test_population);
    toy::Cluster cluster(test_population);
    cluster.test();
    // exit(0);


    cout << "Hell, Toy" << endl;
    srand( (unsigned int) time (NULL) );


    // toy::Prob* prob = new toy::ProbDB("regression.csv");
    toy::Prob* prob = new toy::ProbSim();

    // exploreSamples(prob); // This will finished the program
    // exit(0);
    

    std::ofstream fout("lean_bezier_samples.csv");
    fout << "iter, ";
    fout << "class, ";
    for (int i = 0; i < prob->dim; i++) {
        char c = 'a' + i;
        fout << c << ", ";
    }
    fout << "task, error" << endl;

    // std::ofstream fout2("lean_bezier_seg.csv");
    // fout2 << "iter";
    // for (int i = 0; i < prob->dim; i++) {
    //     char c = 'a' + i;
    //     fout2 << ", " << c << "x" << ", " << c << "y";
    // }
    // fout2 << endl;
    std::ofstream fout2("lean_bezier_line.csv");
    fout2 << "iter, x, y" << endl;

    toy::ModelBezier m(prob->dim);
    cout << m << endl;

    const int MAX_ITER = 1;
    for (int iter = 0; iter < MAX_ITER; iter++) {
        std::vector<toy::Sample> population;
        cout << "== Iter " << iter << " == " << endl;
        const int NPARENTS = 64;
        const int NCHILDS  = 32;
        // for (int i = 0; i < NPARENTS; i++) {
        //     toy::Sample s = m.generate();
        //     prob->query(&s);
        //     population.push_back(s);

        // }

        population = cluster.getCluster(0);

        // loadSample(population);

        // sort(population.begin(), population.end());
        moo_sort(population);

        for (int i = 0; i < population.size(); i++) {
            cout << boost::format("Iter %02d (%02d) : ") % iter % i << population[i] << endl;

            toy::Sample& s = population[i];
            fout << iter << ", ";
            fout << ((i < NCHILDS) ? "G" : "B") << ", ";
            for (int j = 0; j < prob->dim; j++) {
                fout << boost::format("% 6lf, ") % s.params(j);
            }
            fout << boost::format("% 6lf, % 6lf") % s.task % s.error << endl;
        }

        population.erase(population.begin() + NCHILDS, population.end());

        // fit(&m, population);
        fit_bezier(&m, population);

        cout << "== Testing ==" << endl;
        
        // for (double t = 1.0; t <= 2.00001; t += 0.1) {
        for (double t = MIN_TASK; t <= MAX_TASK + 0.00001; t += 0.05) {
            toy::Sample s;
            // s.params = m.alpha + t * m.beta;

            double a = (1 - t) * (1 - t) * (1 - t);
            double b = 3.0 * (1 - t) * (1 - t) * t;
            double c = 3.0 * (1 - t) * t * t;
            double d = t * t * t;
            s.params = a * m.alpha + b * m.beta + c * m.gamma + d * m.delta;

            fout2 << iter;
            for (int i = 0; i < m.dim; i++) {
                s.params(i) = max(-1.0, min(s.params(i), 1.0));
                fout2 << ", " << s.params(i);
            }
            fout2 << endl;
            prob->query(&s);
            cout << t << " --> " << s << endl;
        }

        // Eigen::VectorXd p0 = m.alpha + MIN_TASK * m.beta;
        // Eigen::VectorXd p1 = m.alpha + MAX_TASK * m.beta;
        // fout2 << iter;
        // for (int i = 0; i < prob->dim; i++) {
        //     fout2 << ", " << p0(i);
        // }
        // for (int i = 0; i < prob->dim; i++) {
        //     fout2 << ", " << p1(i);
        // }
        // fout2 << endl;

        

        cout << "Optmized Model = " << endl << m << endl;
        cout << "====================" << endl;
        cout << endl;
    }

    delete prob;

    fout.close();
    

    return 0;
}
