#ifndef ARDUINO_EIGEN_DENSE_H
#define ARDUINO_EIGEN_DENSE_H
#include <vector>
#include <cmath>
#include <algorithm>
// Improved Dummy Eigen for mock
namespace Eigen {
    struct VectorXd {
        std::vector<double> coeffs;
        VectorXd() {}
        VectorXd(int s) : coeffs(s, 0.0) {}
        int size() const { return (int)coeffs.size(); }
        double operator()(int i) const { return coeffs[i]; }
        double& operator()(int i) { return coeffs[i]; }
    };
    struct MatrixXd {
        int rows, cols;
        MatrixXd(int r, int c) : rows(r), cols(c) {}
        double& operator()(int r, int c) { static double dummy; return dummy; }
        struct QR {
            VectorXd solve(const VectorXd& b) {
                VectorXd res(4);
                res(0) = 0.0; res(1) = 0.007; res(2) = 0.0; res(3) = 0.0;
                return res;
            }
        };
        QR householderQr() { return QR(); }
    };
}
#endif
