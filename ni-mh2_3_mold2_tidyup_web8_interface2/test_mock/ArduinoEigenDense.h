#ifndef ARDUINO_EIGEN_DENSE_H
#define ARDUINO_EIGEN_DENSE_H
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace Eigen {
    struct VectorXd {
        std::vector<double> coeffs;
        VectorXd() {}
        VectorXd(int s) : coeffs(s, 0.0) {}
        void resize(int s) { coeffs.assign(s, 0.0); }
        int size() const { return (int)coeffs.size(); }
        double operator()(int i) const { return coeffs[i]; }
        double& operator()(int i) { return coeffs[i]; }
    };

    struct MatrixXd {
        int rows, cols;
        std::vector<double> data;
        MatrixXd(int r, int c) : rows(r), cols(c), data(r * c, 0.0) {}
        double& operator()(int r, int c) { return data[r * cols + c]; }
        double operator()(int r, int c) const { return data[r * cols + c]; }

        struct QR {
            const MatrixXd& A;
            QR(const MatrixXd& m) : A(m) {}

            VectorXd solve(const VectorXd& b) const {
                // Solve A*x = b using Normal Equations: (A^T * A)x = A^T * b
                int n = A.cols;
                int m = A.rows;

                MatrixXd ATA(n, n);
                VectorXd ATb(n);

                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < n; j++) {
                        double sum = 0;
                        for (int k = 0; k < m; k++) {
                            sum += A(k, i) * A(k, j);
                        }
                        ATA(i, j) = sum;
                    }
                    double sumB = 0;
                    for (int k = 0; k < m; k++) {
                        sumB += A(k, i) * b(k);
                    }
                    ATb(i) = sumB;
                }

                // Solve ATA * x = ATb using Gaussian elimination
                for (int i = 0; i < n; i++) {
                    // Pivot
                    int pivot = i;
                    for (int j = i + 1; j < n; j++) {
                        if (std::abs(ATA(j, i)) > std::abs(ATA(pivot, i))) pivot = j;
                    }
                    for (int k = i; k < n; k++) std::swap(ATA(i, k), ATA(pivot, k));
                    std::swap(ATb(i), ATb(pivot));

                    if (std::abs(ATA(i, i)) < 1e-18) continue; // Singular

                    for (int j = i + 1; j < n; j++) {
                        double factor = ATA(j, i) / ATA(i, i);
                        for (int k = i; k < n; k++) {
                            ATA(j, k) -= factor * ATA(i, k);
                        }
                        ATb(j) -= factor * ATb(i);
                    }
                }

                VectorXd x(n);
                for (int i = n - 1; i >= 0; i--) {
                    double sum = 0;
                    for (int j = i + 1; j < n; j++) {
                        sum += ATA(i, j) * x(j);
                    }
                    if (std::abs(ATA(i, i)) > 1e-18)
                        x(i) = (ATb(i) - sum) / ATA(i, i);
                    else
                        x(i) = 0;
                }
                return x;
            }
        };

        QR householderQr() const { return QR(*this); }
    };
}
#endif
