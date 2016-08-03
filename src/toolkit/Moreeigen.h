#ifndef TOOLKIT_MOREIGEN_H
#define TOOLKIT_MOREIGEN_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>

namespace rtql8 {
    namespace toolkit {
        namespace moreeigen {
            Eigen::VectorXd toEigen(const std::vector<double>& v);
            std::vector<double> toStl(const Eigen::VectorXd& v);
            Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& m);
            Eigen::MatrixXd pseudoinverse2(const Eigen::MatrixXd& m);

            Eigen::Vector3d convertStringToVector3d(const char* const str);
            std::string convertVector3dToString(const Eigen::Vector3d& v);

            Eigen::VectorXd convertStringToVectorXd(const char* const str);
            std::string convertVectorXdToString(const Eigen::VectorXd& v);

            class IO {
            public:
                IO(const Eigen::Vector2d& _v) : v(&_v), size(2) {
                }
                IO(Eigen::Vector2d& _v) : v(&_v), size(2) {
                }
                IO(const Eigen::Vector3d& _v) : v(&_v), size(3) {
                }
                IO(Eigen::Vector3d& _v) : v(&_v), size(3) {
                }
                IO(const Eigen::VectorXd& _v) : v(&_v), size(-1) {

                }
                IO(Eigen::VectorXd& _v) : v(&_v), size(-1) {
                }

                IO(Eigen::VectorXd& _v, int sz) : v(&_v), size(sz) {
                }
            
                friend std::ostream& operator << (std::ostream& out, const IO& io) {
                    out << "[";
                    if (io.size == -1) {
                        const Eigen::VectorXd* const v(
                            static_cast<const Eigen::VectorXd* const>(io.v)
                            );
                        if (v->size() > 0) {
                            out << (*v)(0);
                        }
                        for (int i = 1; i < v->size(); i++) {
                            out << ", " << (*v)(i);
                        }
                    } else if (io.size == 3) {
                        const Eigen::Vector3d* const v(
                            static_cast<const Eigen::Vector3d* const>(io.v)
                            );
                        out << (*v)(0);
                        for (int i = 1; i < v->size(); i++) {
                            out << ", " << (*v)(i);
                        }
                    } else if (io.size == 2) {
                        const Eigen::Vector2d* const v(
                            static_cast<const Eigen::Vector2d* const>(io.v)
                            );
                        out << (*v)(0);
                        for (int i = 1; i < v->size(); i++) {
                            out << ", " << (*v)(i);
                        }
                    }
                    out << "]";
                    return out;
                }

                friend std::istream& operator >> (std::istream& in, const IO& io) {
                    char c;
                    in >> c;
                    if (io.size == -1) {
                    	std::stringstream sin;
                    	while(1) {
                    		in >> c;
                    		if (c == ']') break;
                    		sin << c;
                    	}
                        std::vector<double> array;
                        while(true) {
                            double x;
                            sin >> x;
                            if (sin.fail()) {
                                break;
                            }
                            array.push_back(x);
                            sin >> c;
                            if (sin.fail()) break;
//                            if (c == ']') break;
                        }
                        
                        Eigen::VectorXd* v(
                            const_cast<Eigen::VectorXd*>(
                                static_cast<const Eigen::VectorXd* const>(io.v)
                                )
                            );
                        (*v) = toEigen(array);
                    } else if (io.size == 3) {
                        Eigen::Vector3d* v(
                            const_cast<Eigen::Vector3d*>(
                                static_cast<const Eigen::Vector3d* const>(io.v)
                                )
                            );
                        for (int i = 0; i < v->size(); i++) {
                            if (i != 0) in >> c;
                            in >> (*v)(i);
                        }
                        in >> c;
                    } else if (io.size == 2) {
                        Eigen::Vector2d* v(
                            const_cast<Eigen::Vector2d*>(
                                static_cast<const Eigen::Vector2d* const>(io.v)
                                )
                            );
                        for (int i = 0; i < v->size(); i++) {
                            if (i != 0) in >> c;
                            in >> (*v)(i);
                        }
                        in >> c;
                    }
                    return in;
                }
            
            private:
                const void* v;
                int size;
            };

            // class IO
            //////////////////////////////////////////////////

        } // namespace moreeigen
    } // namespace toolkit
} // namespace rtql8


#endif // #ifndef TOOLKIT_MOREIGEN_H

