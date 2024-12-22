/**
 * @file rrt_node.hpp
 * @brief This header file contains the definition of the RRTMainNode class. This ROS2 node is responsible for running
 * the RRT algorithm and publishing the path to the goal to the path-following node.
 */

#pragma once

#include <tf2/utils.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <f1tenth_msgs/msg/raceline_stamped.hpp>
#include <f1tenth_msgs/msg/raceline.hpp>
#include <f1tenth_msgs/msg/obstacle_array.hpp>
#include <f1tenth_msgs/msg/obstacle.hpp>

#include <geos/simplify/DouglasPeuckerSimplifier.h>

#include <opencv2/opencv.hpp>

#include <rrt/rrt.hpp>
#include <rrt/theta_star.hpp>

using namespace std::chrono_literals;
using geos::simplify::DouglasPeuckerSimplifier;


class RRTMainNode : public rclcpp::Node
{
public:
    RRTMainNode();

private:
    rclcpp::Clock::SharedPtr clock;

    const bool USE_OBSTACLE_CALLBACK;
    const std::string MAP_FILEPATH;
    const std::string MAP_YAMLPATH;
    const bool VISUALIZE_ROI;
    const bool VISUALIZE_ROI_ON_MAP;
    const int RACELINE_POINTS_BEFORE;
    const int RACELINE_POINTS_AFTER;
    const int RRT_GOAL_OFFSET;
    const int TRACK_DILATION_SHAPE;
    const int TRACK_DILATION_SIZE;
    const float OBSTACLE_DILATION_SIZE;
    const float LIDAR_OBSTACLE_DILATION_SIZE;
    const float MAX_LIDAR_RANGE;
    const int SCAN_STEP;
    const float MIN_DISTANCE_TO_OBSTACLE;
    const std::string TOPIC_FOR_REAL_POSITION;
    const std::string SCAN_TOPIC;
    const int DISABLE_PRINT_TIMEOUT;

    bool check_min_distance_to_obstacle;
    std::string slow_down_topic;
    bool send_slow_down;

    geometry_msgs::msg::Pose current_pose;
    float real_car_x;
    float real_car_y;
    bool pose_obtained = false;

    f1tenth_msgs::msg::Raceline raceline;
    bool raceline_obtained = false;
    int look_ahead_point_index;
    bool look_ahead_point_index_obtained = false;
    int car_nearest_point_index;
    bool car_nearest_point_index_obtained = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<f1tenth_msgs::msg::ObstacleArray>::SharedPtr obstacle_subscriber_;
    rclcpp::Subscription<f1tenth_msgs::msg::RacelineStamped>::SharedPtr raceline_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr look_ahead_point_index_subscriber_;

	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr odom_publisher_;
    rclcpp::Publisher<f1tenth_msgs::msg::RacelineStamped>::SharedPtr raceline_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slow_down_publisher_;

    std::vector<float> length;
    std::vector<float> cx;
    std::vector<float> cy;
    std::vector<float> heading;
    std::vector<float> curvature;
    std::vector<float> speed;
    std::vector<float> width_right;
    std::vector<float> width_left;
    int num_of_raceline_points;

    int dilation_kernel_shape;

    std::vector<float> rotation_angles;
    std::vector<cv::Mat> rois;
    std::vector<cv::Mat> rois_on_map;
    std::vector<Geometry::Ptr> rois_free_space_polygons;
    std::vector<cv::Mat> translation_matrices;
    std::vector<cv::Mat> inverse_translation_matrices;
    std::vector<cv::Mat> rotation_matrices;
    std::vector<cv::Mat> inverse_rotation_matrices;
    std::vector<cv::Point2f> values_to_subtract;

    cv::Mat current_roi;
    cv::Mat visualized_roi;
    Geometry::Ptr current_free_space;

    cv::Mat map_image;
    cv::Mat original_map_image;
    double resolution;
    double o_x;
    double o_y;
    std::vector<double> left_x, left_y, right_x, right_y;

    cv::Point current_car_pose_in_roi;
    cv::Point2f current_car_pose_in_roi_subpx;
    cv::Point goal_in_roi;
    cv::Point2f goal_in_roi_subpx;
    ThetaStar any_angle_path_calculator;

    int goal_idx;
    float current_yaw;
    float current_speed;

    std::string ax_max_machines_path;
    std::string ggv_path;
    std::vector<std::vector<double>> ax_max_machines;
    std::vector<std::vector<double>> ggv;

    int new_trajectory_end_index;
    bool new_trajectory_just_sent = false;
    bool check_to_send_original_raceline = false;

    GeometryFactory::Ptr factory = GeometryFactory::create();

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void obstacle_callback(const f1tenth_msgs::msg::ObstacleArray::SharedPtr msg);

    void raceline_callback(const f1tenth_msgs::msg::RacelineStamped::SharedPtr raceline_msg);

    void initialize(f1tenth_msgs::msg::Raceline raceline);

    void look_ahead_point_index_callback(const std_msgs::msg::Int16::SharedPtr msg);

    void rrt_main();

    void get_rrt_trajectory(const AnyAnglePath& path,
                            std::vector<float>& length,
                            std::vector<float>& out_x,
                            std::vector<float>& out_y,
                            std::vector<float>& speed,
                            bool& rrt_trajectory_found);

    void calculate_roi(int car_nearest_raceline_index);

    /*
    #########
    # UTILS used only by `roi.cpp`. Don't use them unless you know what you are doing #
    #########
    */
    inline double px_to_m_x(int x_px) const {
        return (double)(x_px - (int)(this->original_map_image.size[1]/2)) * this->resolution + this->o_x;
    }

    inline float subpx_to_m_x(float x_px) const {
        return (x_px - this->original_map_image.size[1]/2) * this->resolution + this->o_x;
    }

    inline double px_to_m_y(int y_px) const {
        return ((double)this->original_map_image.size[0] - (double)(y_px - (int)(this->original_map_image.size[0]/2))) *
            this->resolution + this->o_y;
    }

    inline float subpx_to_m_y(float y_px) const {
        return ((float)this->original_map_image.size[0] - (float)(y_px - this->original_map_image.size[0]/2)) *
            this->resolution + this->o_y;
    }

    inline int m_to_px_x(double x_m) const {
        return my_round(x_m / this->resolution - this->o_x / this->resolution) + (int)(this->original_map_image.size[1]/2);
    }

    inline float m_to_subpx_x(float x_m) const {
        return (x_m - this->o_x) / this->resolution + (this->original_map_image.size[1]/2);
    }

    inline int m_to_px_y(double y_m) const {
        return my_round(-y_m / this->resolution + this->o_y / this->resolution + (double)this->original_map_image.size[0]) +
            (int)(this->original_map_image.size[0]/2);
    }

    inline float m_to_subpx_y(float y_m) const {
        return (-y_m + this->o_y) / this->resolution + (float)this->original_map_image.size[0] + ((float)this->original_map_image.size[0]/2);
    }

    inline int my_round(double value) const {
        return (int)std::nearbyint(value);
    }

    inline cv::Point2d rotate_a_point(cv::Point2d point_to_be_rotated, cv::Point2d car_position, double theta) const {
        point_to_be_rotated -= car_position;
        return cv::Point2d(
            cos(theta) * point_to_be_rotated.x - sin(theta) * point_to_be_rotated.y,
            sin(theta) * point_to_be_rotated.x + cos(theta) * point_to_be_rotated.y
        );
    }

    cv::Point2f from_world_to_roi(cv::Point2f a_point) const;

    cv::Point2f from_world_to_roi_with_index(cv::Point2f a_point, int car_nearest_point_index) const;

    cv::Point2f from_world_to_roi_subpx_with_index(cv::Point2f a_point, int car_nearest_point_index) const;

    cv::Point2f from_roi_to_world(cv::Point2f a_point) const;

    void from_roi_to_world(const std::vector<float>& x_roi,
                           const std::vector<float>& y_roi,
                           std::vector<float>& x_world,
                           std::vector<float>& y_world) const;

    bool is_point_in_free_space(const cv::Mat& roi, geos_t px_x, geos_t px_y, std::string unit, bool verbose = true) const;

    bool is_point_within_roi(const cv::Mat& roi, geos_t x, geos_t y, std::string unit) const;
};
