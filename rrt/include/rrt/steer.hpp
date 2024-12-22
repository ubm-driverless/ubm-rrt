/**
 * @file steer.hpp
 * @brief This header file contains the functions declarations related to computing the trajectory between two RRT nodes
 * and its real cost.
 */

#pragma once

#include <rrt/rrt.hpp>


TrajectoryPtr posq(float x_start, float y_start, float x_goal, float y_goal, float yaw_start, float yaw_goal, float& cost);

bool found_best_x_near(const std::vector<std::reference_wrapper<const RRTNode>>& X_near,
                       const RRTNode& x_rand,
                       std::reference_wrapper<const RRTNode>& x_near,
                       TrajectoryPtr& trajectory,
                       float& cost);

bool found_x_new(const std::vector<std::reference_wrapper<const RRTNode>>& X_near,
                 const RRTNode& x_rand,
                 const AnyAnglePath& path,
                 std::shared_ptr<RRTNode>& x_new);

inline float compute_geodesic_distance(const RRTNode& x1, const RRTNode& x2) {
    return GP::w_e * (x1.get_distance_to_segment() + x2.get_distance_to_segment()) +
           GP::w_theta * (x1.get_yaw_misalignment() + x2.get_yaw_misalignment());
}

inline float compute_geodesic_distance(const RRTNode& x1, const geos_t& x2_distance_to_segment, const float& x2_yaw_misalignment) {
    return GP::w_e * (x1.get_distance_to_segment() + x2_distance_to_segment) +
           GP::w_theta * (x1.get_yaw_misalignment() + x2_yaw_misalignment);
}

inline bool found_trajectory(const RRTNode& start, const RRTNode& goal, TrajectoryPtr& trajectory, float& cost) {
    float x_start = start.get_x();
    float y_start = start.get_y();
    float x_goal = goal.get_x();
    float y_goal = goal.get_y();
    float yaw_start = start.get_yaw();
    float yaw_goal = goal.get_yaw();

    trajectory = posq(x_start, y_start, x_goal, y_goal, yaw_start, yaw_goal, cost);
    return not trajectory->empty();
}
