#ifndef ADVANCED_POLYNOMIAL_FITTER_HPP
#define ADVANCED_POLYNOMIAL_FITTER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <Arduino.h>

class AdvancedPolynomialFitter {
public:
    enum OptimizationMethod {
        NONE,
        GRADIENT_DESCENT,
        LEVENBERG_MARQUARDT,
        NELDER_MEAD
    };

    /**
     * @brief Fits a polynomial using standard Least Squares (Riemann-like sum).
     */
    std::vector<float> fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree);

    /**
     * @brief Fits a polynomial using Lebesgue-measure-weighted Least Squares.
     * Weights points by the width of the interval they represent in the domain.
     * This approximates the L2 integral norm minimization: min \int (P(x) - f(x))^2 dx
     */
    std::vector<float> fitPolynomialLebesgue(const std::vector<float>& x, const std::vector<float>& y, int degree);

    /**
     * @brief Composes two sequential polynomials into one using analytical L2 projection
     * onto a shifted Legendre basis (the "Lebesgue" approach for continuous functions).
     */
    std::vector<float> composePolynomials(const float* p1_coeffs, double p1_delta,
                                          const float* p2_coeffs, double p2_delta, int degree);

private:
    std::vector<double> solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b);
    std::vector<std::vector<double>> getShiftedLegendreMonomialCoeffs(int degree);
    double combinations(int n, int k);
};

#endif // ADVANCED_POLYNOMIAL_FITTER_HPP
