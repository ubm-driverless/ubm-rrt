#include <rrt/steer.hpp>


TrajectoryPtr posq(float x_start, float y_start, float x_goal, float y_goal, float yaw_start, float yaw_goal,
                   float& cost) {

    TrajectoryPtr trajectory = std::make_shared<Trajectory>();

    float l2_cost = 0.0;
    float yaw_cost = 0.0;

    int iteration = 0;
    float rho = (x_goal - x_start) * (x_goal - x_start) + (y_goal - y_start) * (y_goal - y_start);
    float rho_start = rho;

    int max_iterations = sqrt(rho_start) * GP::k_max_num_iterations / (GP::speed * GP::dt);

    float alpha;
    float phi;

    float x = x_start, y = y_start, theta = yaw_start, w, yaw_cost_tmp;
    while (rho > GP::k_gamma and iteration < max_iterations and rho < GP::k_rho * rho_start) {
        alpha = AngleOp::angle_diff(theta, atan2(y_goal - y, x_goal - x));
        // phi = (yaw_goal - alpha) - theta
        phi = AngleOp::angle_diff(theta, AngleOp::angle_diff(alpha, yaw_goal));

        w = GP::k_alpha * alpha + GP::k_phi * phi;
        if (GP::enable_angular_velocity_limit) {
            if (w < -GP::w_max()) {
                w = -GP::w_max();
            }
            if (w > GP::w_max()) {
                w = GP::w_max();
            }
        }

        x = x + cos(theta) * GP::speed * GP::dt;
        y = y + sin(theta) * GP::speed * GP::dt;
        theta = AngleOp::normalize_angle(theta + w * GP::dt);
        rho = (x_goal - x) * (x_goal - x) + (y_goal - y) * (y_goal - y);

        if (iteration == 0) {
            l2_cost += std::sqrt((x - x_start) * (x - x_start) + (y - y_start) * (y - y_start));
            yaw_cost_tmp = 1.0 - AngleOp::cos_angle_diff(theta, yaw_start);
            yaw_cost += yaw_cost_tmp * yaw_cost_tmp;
        } else {
            l2_cost += std::sqrt((x - trajectory->x[iteration - 1]) * (x - trajectory->x[iteration - 1])+
                                 (y - trajectory->y[iteration - 1]) * (y - trajectory->y[iteration - 1]));
            yaw_cost_tmp = 1.0 - AngleOp::cos_angle_diff(theta, trajectory->yaw[iteration - 1]);
            yaw_cost += yaw_cost_tmp * yaw_cost_tmp;
        }

        trajectory->add(x, y, theta);
        iteration++;
    }

    cost = GP::w_d * l2_cost + GP::w_q * yaw_cost;

    if (iteration >= max_iterations or rho >= GP::k_rho * rho_start or
            std::abs(AngleOp::angle_diff(theta, yaw_goal)) > GP::yaw_tolerance) {
        // Trajectory is not valid
        trajectory->clear();
    } else if (GP::downsample_factor > 1) {
        trajectory->downsample(GP::downsample_factor);
    }

    return trajectory;
}


bool found_best_x_near(const std::vector<std::reference_wrapper<const RRTNode>>& X_near,
                       const RRTNode& x_rand,
                       std::reference_wrapper<const RRTNode>& x_near,
                       TrajectoryPtr& trajectory,
                       float& cost) {

    TrajectoryPtr new_trajectory;
    float new_cost;
    cost = std::numeric_limits<float>::max();
    bool found = false;

    for (size_t i = 0; i < X_near.size(); i++) {
        // Steer
        if (not found_trajectory(X_near[i], x_rand, new_trajectory, new_cost)) {
            continue;
        }

        new_cost += X_near[i].get().get_cost() + compute_geodesic_distance(X_near[i], x_rand);

        if (new_cost < cost) {
            trajectory = new_trajectory;
            cost = new_cost;
            x_near = X_near[i];
            found = true;
        }
    }

    return found;
}


bool found_x_new(const std::vector<std::reference_wrapper<const RRTNode>>& X_near,
                 const RRTNode& x_rand,
                 const AnyAnglePath& path,
                 std::shared_ptr<RRTNode>& x_new) {

    std::reference_wrapper<const RRTNode> x_near(RRTNode::get_dummy());

    TrajectoryPtr new_trajectory;
    float new_cost;
    std::tuple<geos_t, geos_t, float> new_last_point;
    float new_yaw_misalignment;

    TrajectoryPtr trajectory;
    float cost = std::numeric_limits<float>::max();
    std::tuple<geos_t, geos_t, float> last_point;
    float yaw_misalignment;

    bool found = false;

    for (size_t i = 0; i < X_near.size(); i++) {
        // Steer
        if (not found_trajectory(X_near[i], x_rand, new_trajectory, new_cost)) {
            continue;
        }

        new_last_point = new_trajectory->pop();
        // For semplicity we assume `closest_segment_idx` and `distance_to_segment` attributes of `x_rand` are the same
        // of candidate `x_new`. This is likely to be true.
        new_yaw_misalignment = 1.0 - AngleOp::cos_angle_diff(std::get<2>(new_last_point), path.yaw[X_near[i].get().get_closest_segment_idx()]);
        new_cost += X_near[i].get().get_cost() + compute_geodesic_distance(X_near[i], x_rand.get_distance_to_segment(), new_yaw_misalignment);

        if (new_cost < cost) {
            trajectory = new_trajectory;
            cost = new_cost;
            last_point = new_last_point;
            yaw_misalignment = new_yaw_misalignment;
            x_near = X_near[i];
            found = true;
        }
    }

    if (found) {
        x_new = std::make_shared<RRTNode>(std::get<0>(last_point), std::get<1>(last_point), std::get<2>(last_point),
                                          x_rand.get_index(), x_rand.get_closest_segment_idx(),
                                          x_rand.get_distance_to_segment(), yaw_misalignment);
        x_new->finalize(cost, x_near.get().get_index(), trajectory);
    }

    return found;
}
