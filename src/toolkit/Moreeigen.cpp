#include "Moreeigen.h"

namespace rtql8 {
    namespace toolkit {
        namespace moreeigen {
            //////////////////////////////////////////////////
            // helper functions

            Eigen::VectorXd toEigen(const std::vector<double>& v) {
                Eigen::VectorXd ret( v.size() );
                for (int i = 0; i < v.size(); i++) {
                    ret(i) = v[i];
                }
                return ret;
            }

            std::vector<double> toStl(const Eigen::Vector3d& v) {
                std::vector<double> ret;
                ret.resize(v.size());
                for (int i = 0; i < v.size(); i++) {
                    ret[i] = v(i);
                }
                return ret;
            }

            Eigen::Matrix3d pseudoinverse(const Eigen::Matrix3d& m) {
                Eigen::JacobiSVD<Eigen::Matrix3d>
                    svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
                Eigen::Matrix3d U = svd.matrixU();
                Eigen::Matrix3d V = svd.matrixV();
                Eigen::Vector3d diag = svd.singularValues();
                Eigen::Matrix3d D = Eigen::Matrix3d::Zero(diag.size(), diag.size());
                D.diagonal() = diag;
                return V * (D.inverse()) * (U.transpose());
            }

            Eigen::Matrix3d pseudoinverse2(const Eigen::Matrix3d& m) {
                const Eigen::Matrix3d mT = m.transpose();
                const Eigen::Matrix3d mTm = mT * m;
                const Eigen::Matrix3d mTmInv = mTm.inverse();
                const Eigen::Matrix3d mTmInvmT = mTmInv * mT;
                return mTmInvmT;
            }

            Eigen::Vector3d convertStringToVector3d(const char* const str) {
                std::stringstream stream("");
                stream << str;
                Eigen::Vector3d ret;
                stream >> IO(ret);
                return ret;
            }

            std::string convertVector3dToString(const Eigen::Vector3d& v) {
                std::stringstream stream("");
                stream << IO(v);
                return stream.str();
            }


            Eigen::VectorXd convertStringToVectorXd(const char* const str) {
                std::stringstream stream("");
                stream << str;
                Eigen::VectorXd ret;
                stream >> IO(ret);
                return ret;


                // std::stringstream stream(str);
                // std::vector<double> temp;
                // while(true) {
                //     double value;
                //     stream >> value;
                //     if (stream.fail()) {
                //         break;
                //     }
                //     temp.push_back(value);
                // }

                // Eigen::VectorXd ret(temp.size());
                // for (int i = 0; i < ret.size(); i++) {
                //     ret(i) = temp[i];
                // }
                // return ret;
            }

            std::string convertVectorXdToString(const Eigen::VectorXd& v) {
                std::stringstream stream("");
                stream << "[";
                for (int i = 0; i < v.size(); i++) {
                    if (i != 0) stream << ' ';
                    stream << v(i);
                }
                stream << "]";
                return stream.str();
            }

            

        } // namespace moreeigen
    } // namespace toolkit
} // namespace rtql8

