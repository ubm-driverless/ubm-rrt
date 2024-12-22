#include <rrt/curvature.hpp>

#include <iostream>

int main() {
    std::vector<float> x = {0.0, 0.3, 1.25, 2.1, 2.85, 3.8, 5.0, 6.4, 8.05, 9.9, 12.05, 14.25, 16.5, 19.25, 21.3, 22.8,
                            23.55, 22.95, 21.35, 19.1};
    std::vector<float> y = {0.0, 0.0, -0.1, -0.9, -2.3, -3.95, -5.75, -7.8, -9.9, -11.6, -12.85, -13.7, -13.8, -13.35,
                            -12.2, -10.5, -8.15, -6.1, -3.95, -1.9};

    std::vector<double> k = compute_curvature(x, y);

    for (auto& value : k) {
        std::cout << value << std::endl;
    }

    return 0;
}
