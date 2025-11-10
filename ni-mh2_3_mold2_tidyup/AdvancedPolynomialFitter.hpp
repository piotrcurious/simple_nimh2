#pragma once

#include <vector>
#include <functional>
#include <ArduinoEigen/Eigen/Dense>

class AdvancedPolynomialFitter {
public:
    enum BoundaryCondition {
        NONE,
        START_ZERO,
        END_ZERO,
        START_END_ZERO
    };

    // Main fitting function with functional parameter for x values
    std::vector<float> fitPolynomialD_superpos5c(
        const std::vector<float>& y,
        std::function<double(int)> x_func,
        int n_points,
        int degree,
        BoundaryCondition bc
    );

    std::vector<float> composePolynomials(
        const float* p1_coeffs, double p1_duration,
        const float* p2_coeffs, double p2_duration,
        int degree
    );
};
