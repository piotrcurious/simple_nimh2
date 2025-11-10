#include "AdvancedPolynomialFitter.hpp"
#include <iostream>

// --- Main fitting function ---
std::vector<float> AdvancedPolynomialFitter::fitPolynomialD_superpos5c(
    const std::vector<float>& y,
    std::function<double(int)> x_func,
    int n_points,
    int degree,
    BoundaryCondition bc)
{
    int n_conditions = 0;
    if (bc == START_ZERO || bc == END_ZERO) {
        n_conditions = 1;
    } else if (bc == START_END_ZERO) {
        n_conditions = 2;
    }

    int n_basis = degree + 1 - n_conditions;
    if (n_basis <= 0) {
        return std::vector<float>(degree + 1, 0.0f);
    }

    Eigen::MatrixXd A(n_points, n_basis);
    Eigen::VectorXd b(n_points);

    for (int i = 0; i < n_points; ++i) {
        b(i) = y[i];
        double x = x_func(i);
        double term = x;
        for (int j = 0; j < n_basis; ++j) {
            if (bc == NONE) {
                A(i, j) = (j == 0) ? 1 : term;
            } else if (bc == START_ZERO) {
                A(i, j) = term;
            } else if (bc == END_ZERO) {
                A(i, j) = term - 1.0;
            } else if (bc == START_END_ZERO) {
                 if (j==0) A(i,j) = term - term*term;
                 else A(i,j) = term*term*pow(term,j-1) - pow(term,j+2);
            }
            term *= x;
        }
    }

    Eigen::VectorXd coeffs_basis = A.colPivHouseholderQr().solve(b);
    Eigen::VectorXd coeffs_final = Eigen::VectorXd::Zero(degree + 1);

    if (bc == NONE) {
        for (int i = 0; i < n_basis; ++i) coeffs_final(i) = coeffs_basis(i);
    } else if (bc == START_ZERO) {
        for (int i = 0; i < n_basis; ++i) coeffs_final(i + 1) = coeffs_basis(i);
    } else if (bc == END_ZERO) {
        coeffs_final(0) = -coeffs_basis.sum();
        for (int i = 0; i < n_basis; ++i) coeffs_final(i + 1) = coeffs_basis(i);
    } else if (bc == START_END_ZERO) {
        if (n_basis > 0) {
            coeffs_final(1) = coeffs_basis(0);
            coeffs_final(2) = -coeffs_basis(0);
        }
        for (int i = 1; i < n_basis; ++i) {
            coeffs_final(i + 1) += coeffs_basis(i);
            coeffs_final(i + 2) -= coeffs_basis(i);
        }
    }

    std::vector<float> result(degree + 1);
    for (int i = 0; i < degree + 1; ++i) {
        result[i] = static_cast<float>(coeffs_final(i));
    }
    return result;
}


// --- Polynomial Composition ---
std::vector<float> AdvancedPolynomialFitter::composePolynomials(
    const float* p1_coeffs, double p1_duration,
    const float* p2_coeffs, double p2_duration,
    int degree)
{
    double total_duration = p1_duration + p2_duration;
    if (total_duration <= 1e-9) { // Avoid division by zero
        return std::vector<float>(p1_coeffs, p1_coeffs + degree + 1);
    }

    double s = p1_duration / total_duration;

    Eigen::VectorXd c1(degree + 1), c2(degree + 1);
    for(int i = 0; i <= degree; ++i) {
        c1(i) = p1_coeffs[i];
        c2(i) = p2_coeffs[i];
    }

    Eigen::MatrixXd T1 = Eigen::MatrixXd::Zero(degree + 1, degree + 1);
    for (int j = 0; j <= degree; ++j) {
        for (int i = 0; i <= j; ++i) {
            T1(i, j) = (i == j ? 1.0 : 0.0) * pow(s, j);
        }
    }

    Eigen::MatrixXd T2 = Eigen::MatrixXd::Zero(degree + 1, degree + 1);
    double s_complement = 1.0 - s;
    for (int j = 0; j <= degree; ++j) {
        for (int i = 0; i <= j; ++i) {
            double binomial_coeff = 1.0;
            for(int k=1; k<=i; ++k) {
                binomial_coeff = binomial_coeff * (j - k + 1) / k;
            }
            T2(i, j) = binomial_coeff * pow(s, j - i) * pow(s_complement, i);
        }
    }

    Eigen::VectorXd p2_eval_at_s(degree + 1);
    for(int i=0; i<=degree; ++i) {
        double val = 0;
        double s_pow = 1;
        for(int j=0; j<=degree; ++j) {
            val += c2(j) * s_pow;
            s_pow *= s;
        }
        p2_eval_at_s(i) = val;
    }


    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(degree + 1, degree + 1);
    double p1_eval_at_1 = c1.sum();
    S(0,0) = p1_eval_at_1;

    Eigen::VectorXd composed_coeffs = T1 * c1 + S * T2 * c2;

    std::vector<float> result(degree + 1);
    for (int i = 0; i <= degree; ++i) {
        result[i] = static_cast<float>(composed_coeffs(i));
    }
    return result;
}
