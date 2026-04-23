#ifndef ARDUINO_EIGEN_DENSE_H
#define ARDUINO_EIGEN_DENSE_H
// Dummy Eigen for mock
namespace Eigen {
    struct VectorXd {
        int size() const {return 1;}
        float operator()(int i) const {return 0;}
        float& operator()(int i) { static float dummy; return dummy; }
        VectorXd() {}
        VectorXd(int s) {}
    };
    struct MatrixXd { MatrixXd(int r, int c){} };
    struct QR { VectorXd solve(VectorXd b) const { return b; } };
    struct HouseholderQR { QR householderQr() const { return QR(); } };
}
#endif
