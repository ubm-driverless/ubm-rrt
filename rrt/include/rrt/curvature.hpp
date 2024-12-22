/**
 * @file curvature.hpp
 * @brief This header file contains the functions implementations for curvature calculation of a 2D trajectory.
 */

#pragma once

#include <vector>
#include <cmath>


std::vector<float> gradient(const std::vector<float>& arr) {
    std::vector<float> grad(arr.size(), 0.0);

    for (size_t i = 1; i < arr.size() - 1; i++) {
        grad[i] = (arr[i + 1] - arr[i - 1]) / 2.0;
    }
    grad[0] = arr[1] - arr[0];
    grad[arr.size() - 1] = arr[arr.size() - 1] - arr[arr.size() - 2];

    return grad;
}


std::vector<double> compute_curvature(const std::vector<float>& x, const std::vector<float>& y) {
    std::vector<float> dx = gradient(x);
    std::vector<float> dy = gradient(y);
    std::vector<float> ddx = gradient(dx);
    std::vector<float> ddy = gradient(dy);

    std::vector<float> ds(x.size(), 0.0);
    for (size_t i = 0; i < x.size(); i++) {
        ds[i] = std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
    }

    std::vector<float> dds = gradient(ds);

    std::vector<double> k(x.size(), 0.0);
    for (size_t i = 0; i < x.size(); i++) {
        // 3.0 instead of 1.5 because `ds` is the square root of dx^2 + dy^2
        k[i] = std::abs(ddx[i] * dy[i] - dx[i] * ddy[i]) / std::pow(ds[i], 3.0);
    }

    return k;
}
