#include <math.h>
#include <tuple>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <chrono>

const float dt = 0.05;
const float k_alpha = 5.0;
const float k_phi = -2.0;
const float k_v = 3.8;
const float k_gamma = 0.01;
const float wb = 0.321;
const float speed = 1;
const float steer_limit = 0.38;
bool enable_angular_velocity_limit = true;
const float k_rho = 1.5;
const float k_max_num_iterations = 1.5;
const float yaw_tolerance = 0.53;
const float w_max = tan(steer_limit) * speed / wb; // 1.24


/**
 * Class to perform operations with angles. The angles are in radians and in the range [-pi, pi).
 */
class AngleOp {
public:
    AngleOp() = delete;                // Delete default constructor
    AngleOp(const AngleOp&) = delete;  // Delete copy constructor
    AngleOp(AngleOp&&) = delete;       // Delete move constructor

    /**
     * Normalize an angle to the range [-pi, pi).
     */
    inline static float normalize_angle(float angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) {
            angle += 2.0 * M_PI;
        }
        return angle - M_PI;
    }

    /**
     * Computes `b - a` in the range [-pi, pi).
     */
    inline static float angle_diff(float a, float b) {
        float diff = std::fmod(b - a + M_PI, 2.0 * M_PI);
        if (diff < 0) {
            diff += 2.0 * M_PI;
        }
        return diff - M_PI;
    }
};


void create_csv(std::vector<std::pair<float, float>> coords, std::string filename)
{
    std::stringstream ss;
    std::ofstream file(filename);

    for (int i = 0; i < coords.size(); i++)
    {
        ss << std::fixed << std::setprecision(5) << coords[i].first << "," << coords[i].second << "\n";
        file << ss.str();
        ss.str("");
    }
    file.close();
}


void create_csv_all_data(std::vector<float> x,
                         std::vector<float> y,
                         std::vector<float> theta,
                         std::vector<float> rho,
                         std::vector<float> alpha,
                         std::vector<float> phi,
                         std::vector<float> w,
                         std::vector<float> t)
{
    std::stringstream ss;
    std::ofstream file("posq_all_data.csv");

    ss << "#x,y,theta,rho,alpha,phi,w,t\n";

    for (int i = 0; i < x.size(); i++) {
        ss << std::fixed << std::setprecision(5) << x[i] << "," << y[i] << "," << theta[i] << "," << rho[i] << ","
           << alpha[i] << "," << phi[i] << "," << w[i] << "," << t[i] << "\n";
        file << ss.str();
        ss.str("");
    }
    file.close();
}


void posq(float x_start, float y_start, float x_goal, float y_goal, float yaw_start, float yaw_goal,
          std::vector<float>& x_out, std::vector<float>& y_out, std::vector<float>& theta_out,
          std::vector<float>& rho_out, std::vector<float>& alpha_out, std::vector<float>& phi_out,
          std::vector<float>& w_out, std::vector<float>& t_out) {

    int iteration = 0;
    float rho = (x_goal - x_start) * (x_goal - x_start) + (y_goal - y_start) * (y_goal - y_start);
    float rho_start = rho;
    int max_iterations = sqrt(rho_start) * k_max_num_iterations / (speed * dt);
    float alpha;
    float phi;
    float t = 0.0;

    x_out.push_back(x_start);
    y_out.push_back(y_start);
    theta_out.push_back(yaw_start);
    rho_out.push_back(rho);
    alpha_out.push_back(-1);
    phi_out.push_back(-1);
    w_out.push_back(0.0);
    t_out.push_back(t);

    float x = x_start, y = y_start, theta = yaw_start, w;
    while (rho > k_gamma and iteration < max_iterations and rho < k_rho * rho_start) {
        alpha = AngleOp::angle_diff(theta, atan2(y_goal - y, x_goal - x));
        phi = AngleOp::angle_diff(theta, AngleOp::angle_diff(alpha, yaw_goal));

        w = k_alpha * alpha + k_phi * phi;
        if (enable_angular_velocity_limit) {
            if (w < -w_max) {
                w = -w_max;
            }
            if (w > w_max) {
                w = w_max;
            }
        }

        x = x + cos(theta) * speed * dt;
        y = y + sin(theta) * speed * dt;
        theta = AngleOp::normalize_angle(theta + w * dt);
        rho = (x_goal - x) * (x_goal - x) + (y_goal - y) * (y_goal - y);
        t += dt;

        x_out.push_back(x);
        y_out.push_back(y);
        theta_out.push_back(theta);
        rho_out.push_back(rho);
        alpha_out.push_back(alpha);
        phi_out.push_back(phi);
        w_out.push_back(w);
        t_out.push_back(t);

        iteration++;
    }

}


int main() {
    auto start = std::chrono::high_resolution_clock::now();

    float x_start = 1.0;
    float y_start = -1.5;
    float x_goal = 0.8;
    float y_goal = -0.9;
    float yaw_start = -0.785398;
    float yaw_goal = M_PI_2;

    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> theta;
    std::vector<float> rho;
    std::vector<float> alpha;
    std::vector<float> phi;
    std::vector<float> w;
    std::vector<float> t;
    posq(x_start, y_start, x_goal, y_goal, yaw_start, yaw_goal, x, y, theta, rho, alpha, phi, w, t);

    auto end = std::chrono::high_resolution_clock::now();
    // Compute the duration
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;

    create_csv_all_data(x, y, theta, rho, alpha, phi, w, t);

    return 0;
}
