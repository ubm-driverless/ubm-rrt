/**
 * @file rrt.hpp
 * @brief This header file contains the general functions, classes and structures used in the Theta*-RRT algorithm.
 */

#pragma once

// Include FLANN headers first to avoid conflicts with OpenCV
// (https://stackoverflow.com/questions/42504592/flann-util-serialization-h-class-stdunordered-mapunsigned-int-stdvectorun)
// (https://github.com/flann-lib/flann/issues/214)
#include <flann/flann.hpp>

#include <random>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <geos.h>
#include <geos/io/WKTReader.h>
#include <geos/io/WKTWriter.h>
#include <geos/operation/buffer/BufferParameters.h>

using namespace std::chrono_literals;
using namespace geos::io;
using geos::operation::buffer::BufferParameters;


#ifdef RRT_CPP_USE_FLOAT_PRECISION
using geos_t = float;
#else
using geos_t = double;
#endif

constexpr size_t ROOT_NODE = std::numeric_limits<size_t>::max();


void convert_output(const std::vector<Geometry::Ptr>& geoms, std::string filename);


// Singleton class to store global parameters
class GP {
public:
    // Delete the copy constructor and assignment operator
    GP(const GP&) = delete;
    GP& operator=(const GP&) = delete;

    // Remember to define the static variables in the source file
    static float strip_width;                    // [m] Width of the strip around the path
    static int n_iterations;                     // [dimensionless] Number of iterations of the RRT algorithm
    static geos_t eps_L2_squared;                // [m^2] Squared Euclidean distance within which two samples are considered equal
    static geos_t delta_R_squared;               // [m^2] Squared Euclidean distance used to define a R^2 ball that contains the nearest neighbors of a sample
    static int max_neighbors;                    // [dimensionless] Maximum number of neighbors to find with radius search
    static geos_t eps_L2_eq;                     // [m] Euclidean distance tolerance to consider two quantities equal (since it's not possible to check floating point numbers for equality)
    static double simplify_tolerance;            // [m] Tolerance used to simplify the generated trajectory and remove unnecessary points
    static int buffer_window;                    // [dimensionless] Number of segments to consider on both sides of the closest segment when buffering a subset of the path for calculating the mean yaw
    static int mean_yaw_window;                  // [dimensionless] Number of segments to consider on both sides of the closest segment when calculating the mean yaw after buffering the path
    static int buffer_retries;                   // [dimensionless] Number of retries to buffer the any-angle path if the random sampled point does not lie on it
    static bool use_rhombus_function;            // [dimensionless] Whether to use the rhombus function to calculate the yaw weights instead of the trapezoidal one
    static float trapezoidal_one_fraction;       // [dimensionless] When using the trapezoidal function, the fraction of the segment that in normal conditions has the value of the function equal to 1.0
    static bool use_orientation_bias;            // [dimensionless] Whether to use the orientation bias when sampling points
    static float orientation_bias;               // [rad] The orientation bias to use when sampling points
    static bool enable_X_free_sampling;          // [dimensionless] Whether to sample uniformly from the free space when generating the random tree
    static int X_free_interval_over_nodes;       // [dimensionless] How many new tree nodes to wait before sampling uniformly from the free space
    static int X_free_interval_over_iterations;  // [dimensionless] How many iterations to wait before sampling uniformly from the free space
    static float goal_region_radius;             // [m] Radius of the goal region
    static float w_e;                            // [dimensionless] Weight for the Euclidean distance in the geodesic distance
    static float w_theta;                        // [dimensionless] Weight for the yaw misalignment in the geodesic distance
    static float w_d;                            // [dimensionless] Weight for the sum of Euclidean distances of the generated trajectory by the steer function
    static float w_q;                            // [dimensionless] Weight for the sum of yaw differences of the generated trajectory by the steer function

    // POSQ parameters
    static float dt;
    static float k_alpha;
    static float k_phi;
    static float k_v;
    static float k_gamma;
    static float wb;
    static float speed;
    static float steer_limit;
    static bool enable_angular_velocity_limit;
    inline static float& w_max() { static float w_max = tan(GP::steer_limit) * GP::speed / GP::wb; return w_max; }
    static float k_rho;
    static float k_max_num_iterations;
    static float yaw_tolerance;
    static int downsample_factor;

    static float drag_coeff;
    static float m_veh;
    static float v_max;
    static float dyn_model_exp;
    static float speed_profile_minimum_start_speed;

    static void assert_params() {
        // TODO: Add all the necessary checks after defining the parameters
        if (strip_width <= 0.0) {
            throw std::runtime_error("The width of the sampling strip must be greater than 0.");
        }
        if (n_iterations <= 0) {
            throw std::runtime_error("The number of iterations must be greater than 0.");
        }
        if (eps_L2_squared <= 0.0) {
            throw std::runtime_error("The squared Euclidean distance tolerance must be greater than 0.");
        }
        if (delta_R_squared <= 0.0) {
            throw std::runtime_error("The squared radius of the ball must be greater than 0.");
        }
        if (eps_L2_squared >= delta_R_squared) {
            throw std::runtime_error("The squared Euclidean distance tolerance must be less than the squared radius of "
                                     "the ball.");
        }
        if (max_neighbors <= 0) {
            throw std::runtime_error("The maximum number of neighbors must be greater than 0.");
        }
        if (eps_L2_eq <= 0.0) {
            throw std::runtime_error("The Euclidean distance tolerance to consider two quantities equal must be "
                                     "greater than 0.");
        }
        if (simplify_tolerance < 0.0) {
            throw std::runtime_error("The tolerance used to simplify the generated trajectory must be greater or equal "
                                     "to 0.");
        }
        if (buffer_window <= 1) {
            throw std::runtime_error("The number of segments to consider on both sides of the closest segment when "
                                     "buffering a subset of the path for calculating the mean yaw must be greater "
                                     "than 1.");
        }
        if (mean_yaw_window <= 0) {
            throw std::runtime_error("The number of segments to consider on both sides of the closest segment when "
                                     "calculating the mean yaw after buffering the path must be greater than 0.");
        }
        if (mean_yaw_window >= buffer_window) {
            throw std::runtime_error("If `mean_yaw_window` is greater or equal to `buffer_window`, the mean yaw "
                                     "calculation will not be precise (if equal) or possible (if greater).");
        }
        if (buffer_retries < 0) {
            throw std::runtime_error("The number of retries to buffer the any-angle path must be greater or equal to 0.");
        }
        if (not (trapezoidal_one_fraction > 0.0 and trapezoidal_one_fraction < 1.0)) {
            throw std::runtime_error("The fraction of the segment that in normal conditions has the value of the "
                                     "trapezoidal function equal to 1.0 must be in the range (0, 1).");
        }
        if (not (orientation_bias > 0.0 and orientation_bias < M_PI * 0.5)) {
            throw std::runtime_error("The orientation bias must be in the range (0, pi/2).");
        }
        if (X_free_interval_over_nodes <= 0) {
            throw std::runtime_error("The number of new tree nodes to wait before sampling uniformly from the free space "
                                     "must be greater than 0.");
        }
        if (X_free_interval_over_iterations <= 0) {
            throw std::runtime_error("The number of iterations to wait before sampling uniformly from the free space "
                                     "must be greater than 0.");
        }
    }

    static GP& get_obj() {
        static GP instance;
        return instance;
    }

private:
    // Make the constructor private so that the class cannot be instantiated outside
    GP() {}
};


struct AnyAnglePath {
public:
    using Ptr = std::unique_ptr<AnyAnglePath>;

    std::vector<geos_t> x;
    std::vector<geos_t> y;
    std::vector<geos_t> segment_length;
    std::vector<float> yaw;
    float initial_yaw;
    float final_yaw;

    /**
     * Get the number of points in the path.
     * @warning Use `sgms_size()` instead to get the number of points in the path.
     */
    inline size_t pts_size() const {
        return x.size();
    }

    /**
     * Get the number of segments in the path.
     * @warning Use `pts_size()` instead to get the number of points in the path.
     */
    inline size_t sgms_size() const {
        return segment_length.size();
    }

    inline static AnyAnglePath::Ptr create(const std::vector<geos_t>& x,
                                           const std::vector<geos_t>& y,
                                           const std::vector<geos_t>& segment_length,
                                           const std::vector<float>& yaw,
                                           const float& initial_yaw,
                                           const float& final_yaw) {
        return std::make_unique<AnyAnglePath>(x, y, segment_length, yaw, initial_yaw, final_yaw);
    }

    AnyAnglePath(const std::vector<geos_t>& x,
                 const std::vector<geos_t>& y,
                 const std::vector<geos_t>& segment_length,
                 const std::vector<float>& yaw,
                 const float& initial_yaw,
                 const float& final_yaw)
            : x(x), y(y), segment_length(segment_length), yaw(yaw), initial_yaw(initial_yaw), final_yaw(final_yaw) {}

    // INFO: the first input of this function depends on the any angle path implementation
    static AnyAnglePath::Ptr process_theta_star_output(const std::list<std::tuple<int, int>>& path,
                                                       const float& initial_yaw,
                                                       const float& final_yaw) {
        std::list<std::tuple<float, float>> path_float;
        for (const auto& p : path) {
            path_float.emplace_back(static_cast<float>(std::get<0>(p)), static_cast<float>(std::get<1>(p)));
        }
        return process_theta_star_output(path_float, initial_yaw, final_yaw);
    }

    static AnyAnglePath::Ptr process_theta_star_output(const std::list<std::tuple<float, float>>& path,
                                                       const float& initial_yaw,
                                                       const float& final_yaw) {
        std::vector<geos_t> x;
        std::vector<geos_t> y;
        std::vector<geos_t> segment_length;
        std::vector<float> yaw;

        x.reserve(path.size());
        y.reserve(path.size());
        segment_length.reserve(path.size() - 1);
        yaw.reserve(path.size() - 1);

        geos_t dx;
        geos_t dy;

        auto it = path.begin();
        auto it_next = path.begin();
        it_next++;

        for (; it_next != path.end(); ++it, ++it_next) {
            x.push_back(static_cast<geos_t>(std::get<0>(*it)));
            y.push_back(static_cast<geos_t>(std::get<1>(*it)));
            dx = static_cast<geos_t>(std::get<0>(*it_next) - std::get<0>(*it));
            dy = static_cast<geos_t>(std::get<1>(*it_next) - std::get<1>(*it));
            segment_length.push_back(std::sqrt(dx * dx + dy * dy));
            yaw.push_back(std::atan2(dy, dx));
        }

        // Push the last point
        x.push_back(static_cast<geos_t>(std::get<0>(*it)));
        y.push_back(static_cast<geos_t>(std::get<1>(*it)));

        return AnyAnglePath::create(x, y, segment_length, yaw, initial_yaw, final_yaw);
    }

    friend std::ostream& operator<<(std::ostream& os, const AnyAnglePath& path) {
        os << "AnyAnglePath: \n";
        os << "x: ";
        for (const auto& val : path.x) {
            os << val << " ";
        }
        os << "\n";

        os << "y: ";
        for (const auto& val : path.y) {
            os << val << " ";
        }
        os << "\n";

        os << "segment_length: ";
        for (const auto& val : path.segment_length) {
            os << val << " ";
        }
        os << "\n";

        os << "yaw: ";
        for (const auto& val : path.yaw) {
            os << val << " ";
        }
        os << "\n";

        os << "initial_yaw: " << path.initial_yaw << "\n";
        os << "final_yaw: " << path.final_yaw << "\n";

        return os;
    }
};


struct Trajectory {
    std::vector<geos_t> x;
    std::vector<geos_t> y;
    std::vector<float> yaw;

    inline void add(const geos_t& x, const geos_t& y, const float& yaw) {
        this->x.push_back(x);
        this->y.push_back(y);
        this->yaw.push_back(yaw);
    }

    std::tuple<geos_t, geos_t, float> pop() {
        if (empty()) {
            throw std::runtime_error("The trajectory is empty. Cannot pop");
        }
        geos_t x = this->x.back();
        geos_t y = this->y.back();
        float yaw = this->yaw.back();
        this->x.pop_back();
        this->y.pop_back();
        this->yaw.pop_back();
        return std::make_tuple(x, y, yaw);
    }

    inline void clear() {
        x.clear();
        y.clear();
        yaw.clear();
        // TODO: remove this assertion. Just for debugging purposes
        assert(empty());
    }

    inline bool empty() const {
        int n_empty = 0;
        n_empty += x.empty();
        n_empty += y.empty();
        n_empty += yaw.empty();
        if (n_empty == 0) {
            return false;
        } else if (n_empty == 3) {
            return true;
        } else {
            throw std::runtime_error("The trajectory is in an inconsistent state.");
        }
    }

    void downsample(int factor) {
        // Assuming factor > 1

        std::vector<geos_t> new_x;
        std::vector<geos_t> new_y;
        std::vector<float> new_yaw;

        for (size_t i = 0; i < x.size(); i += factor) {
            new_x.push_back(x[i]);
            new_y.push_back(y[i]);
            new_yaw.push_back(yaw[i]);
        }

        x = std::move(new_x);
        y = std::move(new_y);
        yaw = std::move(new_yaw);
    }

    Trajectory() = default;
};

using TrajectoryPtr = std::shared_ptr<Trajectory>;


struct RRTNode {
public:
    RRTNode(geos_t x, geos_t y, float yaw, size_t index, int closest_segment_idx,
            geos_t distance_to_segment, float yaw_misalignment)
            : x(x), y(y), yaw(yaw), index(index), closest_segment_idx(closest_segment_idx),
              distance_to_segment(distance_to_segment), yaw_misalignment(yaw_misalignment) {
        cost = -1.0;
    }

    void finalize(const float& cost, const size_t& parent, TrajectoryPtr trajectory) {
        if (is_final()) {
            throw std::runtime_error("The node has already been finalized.");
        }
        this->cost = cost;
        this->parent = parent;
        this->trajectory = trajectory;
    }

    void set_to_root() {
        assert(index == 0);
        parent = ROOT_NODE;
        cost = 0.0;
    }

    inline bool is_root() const {
        return index == 0 and parent == ROOT_NODE;
    }

    inline bool is_final() const {
        return cost >= 0.0 or index == 0;
    }

    static const RRTNode& get_dummy() {
        static RRTNode dummy;  // index = 0, parent = 0, cost = 0.0 and trajectory = empty
        return dummy;
    }

    inline const geos_t& get_x() const { return x; }
    inline const geos_t& get_y() const { return y; }
    inline const float& get_yaw() const { return yaw; }
    inline const float& get_cost() const { return cost; }
    inline const size_t& get_index() const { return index; }
    inline const int& get_closest_segment_idx() const { return closest_segment_idx; }
    inline const geos_t& get_distance_to_segment() const { return distance_to_segment; }
    inline const float& get_yaw_misalignment() const { return yaw_misalignment; }
    inline const size_t& get_parent() const { return parent; }
    inline const Trajectory& get_trajectory() const { return *trajectory; }

private:
    geos_t x;
    geos_t y;
    float yaw;
    float cost;
    size_t index;                // Index of the node in the RRT tree vector
    int closest_segment_idx;     // Index of the closest segment in the path
    geos_t distance_to_segment;  // Distance to the closest segment
    float yaw_misalignment;      // Value to represent the yaw misalignment with the closest segment
    size_t parent;               // For the root node, `parent = ROOT_NODE = std::numeric_limits<size_t>::max()`
    TrajectoryPtr trajectory;

    RRTNode() = default;
};


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

    /**
     * Finds the angle bisector between two angles. The inputs can be given in any order and they don't need to be
     * normalized. The output is normalized.
     */
    inline static float bisect_angle(float a, float b) {
        return normalize_angle(a + angle_diff(a, b) * 0.5);
    }

    /**
     * Computes the weighted circular mean of a set of angles, given their respective weights. The inputs don't need to
     * be normalized. The output is in the range [-pi, pi).
     */
    static float weighted_circular_mean(const std::vector<float>& angles, const std::vector<float>& weights) {
        if (angles.size() != weights.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("rrt_cpp"), "The number of angles and weights must be the same.");
            throw std::runtime_error("The number of angles and weights must be the same.");
        }

        float sum_sin = 0.0;
        float sum_cos = 0.0;

        for (size_t i = 0; i < angles.size(); i++) {
            sum_sin += std::sin(angles[i]) * weights[i];
            sum_cos += std::cos(angles[i]) * weights[i];
        }

        // The `atan2` function returns a value in the range [-pi, pi] and not [-pi, pi), so we need to normalize it
        return normalize_angle(std::atan2(sum_sin, sum_cos));
    }

    /**
     * Computes the cosine of the angle difference between two angles. The inputs don't need to be normalized.
     * The order of the angles doesn't matter, since `cos(a - b) = cos(b - a)`.
     */
    inline static float cos_angle_diff(float a, float b) {
        return std::cos(angle_diff(a, b));
    }
};
