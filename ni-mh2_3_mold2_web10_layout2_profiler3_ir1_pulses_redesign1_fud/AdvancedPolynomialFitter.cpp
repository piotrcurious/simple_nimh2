#include "AdvancedPolynomialFitter.hpp"

std::vector<float> AdvancedPolynomialFitter::fitPolynomial(const std::vector<float>& x, const std::vector<float>& y, int degree) {
    if (x.size() != y.size() || x.empty() || degree < 1) return {};
    size_t n = x.size(); size_t m = degree + 1;
    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);
    for (size_t i = 0; i < n; ++i) {
        std::vector<double> xi_powers(m); xi_powers[0] = 1.0;
        for (size_t j = 1; j < m; ++j) xi_powers[j] = xi_powers[j - 1] * x[i];
        for (size_t j = 0; j < m; ++j) {
            ATy[j] += xi_powers[j] * y[i];
            for (size_t k = 0; k < m; ++k) ATA[j][k] += xi_powers[j] * xi_powers[k];
        }
    }
    std::vector<double> coeffs = solveLinearSystem(ATA, ATy);
    return std::vector<float>(coeffs.begin(), coeffs.end());
}

std::vector<float> AdvancedPolynomialFitter::fitPolynomialLebesgue(const std::vector<float>& x, const std::vector<float>& y, int degree) {
    if (x.size() != y.size() || x.empty() || degree < 1) return {};
    size_t n = x.size(); size_t m = degree + 1;

    // Sort indices by X for measure calculation
    std::vector<size_t> idx(n);
    for(size_t i=0; i<n; ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b){ return x[a] < x[b]; });

    std::vector<std::vector<double>> ATA(m, std::vector<double>(m, 0.0));
    std::vector<double> ATy(m, 0.0);

    for (size_t i = 0; i < n; ++i) {
        size_t curr = idx[i];

        // Calculate "Lebesgue measure" weight for this point.
        // Approx: dx_i = (x_{i+1} - x_{i-1}) / 2
        double weight = 0;
        if (n == 1) weight = 1.0;
        else if (i == 0) weight = (x[idx[1]] - x[idx[0]]);
        else if (i == n - 1) weight = (x[idx[n-1]] - x[idx[n-2]]);
        else weight = (x[idx[i+1]] - x[idx[i-1]]) / 2.0;

        if (weight < 0) weight = 0; // Should not happen if sorted

        std::vector<double> xi_powers(m); xi_powers[0] = 1.0;
        for (size_t j = 1; j < m; ++j) xi_powers[j] = xi_powers[j - 1] * x[curr];

        for (size_t j = 0; j < m; ++j) {
            ATy[j] += weight * xi_powers[j] * y[curr];
            for (size_t k = 0; k < m; ++k) ATA[j][k] += weight * xi_powers[j] * xi_powers[k];
        }
    }
    std::vector<double> coeffs = solveLinearSystem(ATA, ATy);
    return std::vector<float>(coeffs.begin(), coeffs.end());
}

std::vector<double> AdvancedPolynomialFitter::solveLinearSystem(std::vector<std::vector<double>>& A, std::vector<double>& b) {
    size_t n = A.size();
    for (size_t k = 0; k < n; ++k) {
        size_t max_row = k;
        for (size_t i = k + 1; i < n; ++i) if (std::abs(A[i][k]) > std::abs(A[max_row][k])) max_row = i;
        std::swap(A[k], A[max_row]); std::swap(b[k], b[max_row]);
        if (std::abs(A[k][k]) < 1e-18) continue;
        for (size_t i = k + 1; i < n; ++i) {
            double factor = A[i][k] / A[k][k];
            for (size_t j = k; j < n; ++j) A[i][j] -= factor * A[k][j];
            b[i] -= factor * b[k];
        }
    }
    std::vector<double> x(n, 0.0);
    for (int i = n - 1; i >= 0; --i) {
        double sum = 0;
        for (size_t j = i + 1; j < n; ++j) sum += A[i][j] * x[j];
        if (std::abs(A[i][i]) > 1e-18) x[i] = (b[i] - sum) / A[i][i];
    }
    return x;
}

double AdvancedPolynomialFitter::combinations(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1.0;
    if (k > n / 2) k = n - k;
    double res = 1.0;
    for (int i = 1; i <= k; ++i) res = res * (n - i + 1) / i;
    return res;
}

std::vector<std::vector<double>> AdvancedPolynomialFitter::getShiftedLegendreMonomialCoeffs(int degree) {
    std::vector<std::vector<double>> L(degree + 1);
    L[0] = {1.0};
    if (degree > 0) L[1] = {-1.0, 2.0};
    for (int j = 1; j < degree; ++j) {
        int jp1 = j + 1, tjp1 = 2 * j + 1;
        L[jp1].resize(j + 2, 0.0);
        for (int k = 0; k <= j; ++k) L[jp1][k + 1] = tjp1 * 2.0 * L[j][k];
        for (int k = 0; k <= j; ++k) L[jp1][k] -= tjp1 * L[j][k];
        for (int k = 0; k < j; ++k) L[jp1][k] -= j * L[j - 1][k];
        for (int k = 0; k <= jp1; ++k) L[jp1][k] /= jp1;
    }
    return L;
}

std::vector<float> AdvancedPolynomialFitter::composePolynomials(const float* p1_coeffs, double p1_delta, const float* p2_coeffs, double p2_delta, int degree) {
    double total_delta = p1_delta + p2_delta;
    int m = degree + 1;
    if (total_delta <= 0) return std::vector<float>(m, 0.0f);
    double w1 = p1_delta / total_delta, w2 = 1.0 - w1;
    std::vector<double> p1d(m), p2d(m);
    for (int i = 0; i < m; ++i) { p1d[i] = p1_coeffs[i]; p2d[i] = p2_coeffs[i]; }
    auto leg_mono = getShiftedLegendreMonomialCoeffs(degree);
    std::vector<double> leg_coeffs(m, 0.0);
    for (int j = 0; j <= degree; ++j) {
        const auto& Lj = leg_mono[j];
        double integral_1 = 0.0;
        for (int k = 0; k <= degree; ++k) {
            for (int l = 0; l <= j; ++l) integral_1 += p1d[k] * Lj[l] * (std::pow(w1, l + 1) / (k + l + 1.0));
        }
        std::vector<double> qj(j + 1, 0.0);
        for (int l = 0; l <= j; ++l) {
            for (int mm = 0; mm <= l; ++mm) qj[mm] += Lj[l] * combinations(l, mm) * std::pow(w1, l - mm) * std::pow(w2, mm);
        }
        double integral_2 = 0.0;
        for (int k = 0; k <= degree; ++k) {
            for (int mm = 0; mm <= j; ++mm) integral_2 += p2d[k] * qj[mm] / (k + mm + 1.0);
        }
        integral_2 *= w2;
        leg_coeffs[j] = (integral_1 + integral_2) * (2.0 * j + 1.0);
    }
    std::vector<double> resd(m, 0.0);
    for (int k = 0; k <= degree; ++k) {
        for (int j = k; j <= degree; ++j) resd[k] += leg_coeffs[j] * leg_mono[j][k];
    }
    return std::vector<float>(resd.begin(), resd.end());
}
