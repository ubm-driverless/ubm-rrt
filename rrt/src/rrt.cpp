#include <rrt/rrt.hpp>
#include <rrt/rrt_node.hpp>
#include <rrt/nn_search.hpp>
#include <rrt/mean_yaw.hpp>
#include <rrt/steer.hpp>
#include <rrt/curvature.hpp>
#include <rrt/speed_profile.hpp>


// NOTE: remember that when the RRT routine finished generating the new trajectory, the car won't be at the start point.


void RRTMainNode::get_rrt_trajectory(const AnyAnglePath& path,
                                     std::vector<float>& length,
                                     std::vector<float>& out_x,
                                     std::vector<float>& out_y,
                                     std::vector<float>& speed,
                                     bool& rrt_trajectory_found) {

    // INFO: remember that we are in the ROI pixel frame even though the units are meters

    /*
     * --- Initialization ---
     */

    auto start = std::chrono::high_resolution_clock::now();

    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()

    const GeometryFactory::Ptr factory = GeometryFactory::create();



    /*
     *  --- Create the strip around the path ---
     */

    // dims = 2 because we are working with 2D points
    CoordinateSequence points(/*size=*/0, /*dims=*/2);

    for (size_t i = 0; i < path.pts_size(); i ++) {
        points.add(path.x[i], path.y[i]);
    }

    // TODO: consider using prepared geometries (e.g., PreparedLineString) to speed up the code
    //  (https://trac.osgeo.org/geos/wiki/PreparedGeometry)
    LineString::Ptr g_line = factory->createLineString(points);
    Geometry::Ptr g_strip = g_line->buffer(GP::strip_width / 2, /*quadrantSegments=*/4,
                                           BufferParameters::CAP_FLAT);  // Instead of CAP_ROUND

    // After the intersection we are not checking if the start and goal points are in the strip.
    // They should be since in `obstacle.cpp` we check if the car pose and goal are in the free space.
    // In any case, even if they are not, the RRT algorithm will simply not find a trajectory.
    const Geometry& roi_polygon = *current_free_space;
    g_strip = g_strip->intersection(&roi_polygon);
    if (g_strip->isEmpty()) {
        RCLCPP_ERROR(get_logger(), "Strip after intersection with ROI is empty. Something went wrong.");
        return;
    }

    // Get the rectangle that bounds the strip
    const Envelope* envelope;  // Variable freed automatically by the factory, don't call `delete` on it
    envelope = g_strip->getEnvelopeInternal();
    if (envelope->isNull()) {
        throw std::runtime_error("Envelope is null. Something went wrong.");
    }
    if (not envelope->isFinite()) {
        throw std::runtime_error("Envelope is not finite. Something went wrong.");
    }

    std::pair<geos_t, geos_t> envelope_min = {envelope->getMinX(), envelope->getMinY()};
    std::pair<geos_t, geos_t> envelope_max = {envelope->getMaxX(), envelope->getMaxY()};



    /*
     *  --- RRT main ---
     */

    std::uniform_real_distribution<> dis_x(envelope_min.first, envelope_max.first);
    std::uniform_real_distribution<> dis_y(envelope_min.second, envelope_max.second);
    std::uniform_real_distribution<> dis_yaw(-GP::orientation_bias, GP::orientation_bias);

    // Create the FLANN KD-tree
    flann::Matrix<geos_t> dataset(new geos_t[2], 1, 2);
    dataset[0][0] = path.x[0];
    dataset[0][1] = path.y[0];
    flann::Matrix<int> k_indices(new int[GP::max_neighbors], 1, GP::max_neighbors);
	flann::Matrix<geos_t> k_distances(new geos_t[GP::max_neighbors], 1, GP::max_neighbors);
    flann::Matrix<geos_t> query(new geos_t[2], 1, 2);

    // `flann::KDTreeIndexParams(1)` is the number of trees to use
    // It doesn't make sense to use more than one tree for exact search (according to the FLANN stdout message)
    flann::Index<flann::L2_Simple<geos_t>> kd_tree(dataset, flann::KDTreeIndexParams(1));
    kd_tree.buildIndex();

    // Create the RRT tree
    std::vector<RRTNode> rrt_tree;
    rrt_tree.emplace_back(path.x[0], path.y[0], path.initial_yaw, 0, 0, 0.0,
                          1.0 - AngleOp::cos_angle_diff(path.initial_yaw, path.yaw[0]));
    rrt_tree[0].set_to_root();

    bool trajectory_found = false;
    bool free_sampling_switch;
    int n_nodes = 1;

    for (int i = 0; i < GP::n_iterations; i++) {
        free_sampling_switch = (n_nodes % GP::X_free_interval_over_nodes == 0 or
                                (i + 1) % GP::X_free_interval_over_iterations == 0);

        // At the first iteration, sample the goal point. At the following iterations, sample a random point
        CoordinateXY coord_rand;
        if (i == 0) {
            coord_rand = CoordinateXY(path.x[path.pts_size() - 1], path.y[path.pts_size() - 1]);
        } else {
            coord_rand = CoordinateXY(dis_x(gen), dis_y(gen));
            Point::Ptr point_rand = factory->createPoint(coord_rand);

            if (GP::enable_X_free_sampling and free_sampling_switch) {
                if (not point_rand->within(&roi_polygon)) {
                    continue;
                }
            } else if (not point_rand->within(g_strip.get())) {
                continue;
            }
        }

        // Find X_near set according to Euclidean distance using nearest neighbor search,
        // but if point_rand is too close to any point in the tree, skip this iteration,
        // because it will have a similar yaw to the already existing point
        std::vector<std::reference_wrapper<const RRTNode>> X_near;
        std::reference_wrapper<const RRTNode> closest_point(RRTNode::get_dummy());

        query[0][0] = coord_rand.x;
        query[0][1] = coord_rand.y;
        // TODO: check if it is faster to sample the yaw first and then check if the point is already in the tree
        if (is_already_inserted_xy_flann_tree(kd_tree, query, k_indices, k_distances, rrt_tree,
                                              X_near, closest_point)) {
            continue;
        }

        float final_yaw;
        float mean_yaw;
        int idx_closest;
        geos_t distance_to_coord;
        if (not found_mean_yaw(coord_rand, points, path, *factory, mean_yaw,
                               idx_closest, distance_to_coord)) {
            continue;
        }
        if (GP::use_orientation_bias) {
            final_yaw = mean_yaw + dis_yaw(gen);
        } else {
            final_yaw = mean_yaw;
        }

        float yaw_misalignment = 1.0 - AngleOp::cos_angle_diff(final_yaw, path.yaw[idx_closest]);
        RRTNode x_rand(coord_rand.x, coord_rand.y, final_yaw, n_nodes, idx_closest,
                       distance_to_coord, yaw_misalignment);

        // If X_near has at most 1 element, use closest_point as x_near; otherwise, choose the point in X_near with the lowest cost.
        // Take the last point of the generated `Trajectory` as the new node of the tree. Call it `x_new`. This is to avoid
        // artifacts (jumps) in the generated trajectory.
        std::shared_ptr<RRTNode> x_new;
        if (X_near.size() == 0) {
            X_near.push_back(closest_point);
        }
        if (not found_x_new(X_near, x_rand, path, x_new)) {
            continue;
        }

        // Check collisions with ROI
        bool collision_detected = false;
        for (size_t j = 0; j < x_new->get_trajectory().x.size(); j++) {
            if (not is_point_in_free_space(current_roi, x_new->get_trajectory().x[j], x_new->get_trajectory().y[j], "meter")) {
                collision_detected = true;
                break;
            }
        }
        if (collision_detected) {
            continue;
        }

        // Add x_new to the two tree data structures
        query[0][0] = x_new->get_x();
        query[0][1] = x_new->get_y();
        add_point_to_flann_tree(kd_tree, query);
        rrt_tree.push_back(std::move(*x_new));
        // Keep the following line here (i.e., after adding the nodes) because `n_nodes` starts from 1
        n_nodes++;

        // If x_new belongs to the goal region (we are not checking yaw similarity), break the loop and get the final trajectory
        float distance_to_goal = std::sqrt(std::pow(path.x[path.pts_size() - 1] - x_new->get_x(), 2) +
                                           std::pow(path.y[path.pts_size() - 1] - x_new->get_y(), 2));
        if (distance_to_goal < GP::goal_region_radius) {
            trajectory_found = true;
            break;
        }
    }

    delete[] dataset.ptr();
    delete[] k_indices.ptr();
    delete[] k_distances.ptr();
    delete[] query.ptr();

    // If trajectory not found, return
    if (not trajectory_found) {
        return;
    }

    // Get the final trajectory
    std::list<float> traj_x, traj_y;
    std::reference_wrapper<const RRTNode> current_node = rrt_tree.back();
    std::vector<cv::Point2f> junction_points;
    while (not current_node.get().is_root()) {
		geos_t current_junction_x, current_junction_y;
        current_junction_x = current_node.get().get_x();
        current_junction_y = current_node.get().get_y();
        if (VISUALIZE_ROI) {
            junction_points.push_back(cv::Point2f(current_junction_x, current_junction_y));
        }

        traj_x.push_front(current_junction_x);
        traj_y.push_front(current_junction_y);

        traj_x.insert(traj_x.begin(), current_node.get().get_trajectory().x.begin(), current_node.get().get_trajectory().x.end());
        traj_y.insert(traj_y.begin(), current_node.get().get_trajectory().y.begin(), current_node.get().get_trajectory().y.end());

        current_node = rrt_tree[current_node.get().get_parent()];
    }
    traj_x.push_front(path.x[0]);
    traj_y.push_front(path.y[0]);

    std::vector<float> traj_x_v(traj_x.begin(), traj_x.end());
    std::vector<float> traj_y_v(traj_y.begin(), traj_y.end());

    // Divide x and y by resolution to get the trajectory in pixels in the ROI frame
    for (size_t i = 0; i < traj_x_v.size(); i++) {
        traj_x_v[i] /= resolution;
        traj_y_v[i] /= resolution;
    }

	if (VISUALIZE_ROI) {
        double scale_factor = 10.0; // Change this to your desired scale. >1.0 for upscaling, <1.0 for downscaling.
        cv::resize(visualized_roi, visualized_roi, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);

        // Convert the trajectory to a compatible format for visualization
        std::vector<cv::Point> points;
        for (size_t i = 0; i < traj_x_v.size(); i++) {
            points.push_back(cv::Point(traj_x_v[i] * scale_factor, traj_y_v[i] * scale_factor));
        }

        cv::polylines(visualized_roi, points, /*isClosed=*/false, cv::Scalar(255, 0, 0), /*thickness=*/1);

        for (const auto& junction_point : junction_points) {
            cv::circle(visualized_roi, junction_point / resolution * scale_factor, /*radius=*/scale_factor / 3,
                       cv::Scalar(0, 165, 255), /*thickness=*/-1);
        }

        cv::imshow("ROI [rrt]", visualized_roi);
        cv::waitKey(20);
    }

    // Rotate back the generated trajectory
    from_roi_to_world(traj_x_v, traj_y_v, out_x, out_y);

    // Prepare the inputs for the speed profile calculation
    length.resize(out_x.size());
    length[0] = 0.0;  // The first element is always 0

    float cumulated_distance = 0.0;
    for (size_t i = 1; i < out_x.size(); i++) {
        float dx = out_x[i] - out_x[i - 1];
        float dy = out_y[i] - out_y[i - 1];
        cumulated_distance += std::sqrt(dx * dx + dy * dy);
        length[i] = cumulated_distance;
    }

    // Calculate speed profile
    // TODO: the TUM output should be scaled down? Not sure, but I don't think so since we are in a meters frame
    std::vector<double> kappa = compute_curvature(out_x, out_y);

    double v_start;
    if (current_speed < GP::speed_profile_minimum_start_speed) {
        v_start = GP::speed_profile_minimum_start_speed;
    } else {
        v_start = current_speed;
    }
    speed = calc_vel_profile(ax_max_machines,
                             kappa,
                             std::vector<double>(length.begin(), length.end()),
                             GP::drag_coeff,
                             GP::m_veh,
                             ggv,
                             GP::v_max,
                             GP::dyn_model_exp,
                             std::vector<double>(),
                             v_start,
                             this->speed[goal_idx]);

    rrt_trajectory_found = true;

    // End time
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_DEBUG(get_logger(), "Elapsed time: %f s", elapsed.count());
}


void RRTMainNode::rrt_main() {
    // Called right at the end of `obstacle_callback` or `scan_callback`
    // ros2 topic pub -t 1 /obstacle f1tenth_msgs/msg/ObstacleArray

    // Calculate Theta* path
    std::list<tuple<int, int>> aap = any_angle_path_calculator.thetaStar(
            {current_car_pose_in_roi.x, current_car_pose_in_roi.y},
            {goal_in_roi.x, goal_in_roi.y},
            rois[car_nearest_point_index]
    );

    if (aap.empty()) {
        RCLCPP_WARN(get_logger(), "Theta* path is empty");
        return;
    }

    float initial_yaw = AngleOp::angle_diff(tf2::getYaw(current_pose.orientation), rotation_angles[car_nearest_point_index]);
    // Added M_PI_2 because of (most likely) reference system differences with TUM repo (could also simply be the cause of the coordinate system mess)
    // This is not true anymore thanks to our fixes to the raceline repo. The heading is now correct.
    // float goal_yaw = AngleOp::angle_diff(heading[goal_idx] + M_PI_2, rotation_angles[car_nearest_point_index]);
    float goal_yaw = AngleOp::angle_diff(heading[goal_idx], rotation_angles[car_nearest_point_index]);

    if (VISUALIZE_ROI) {
        cv::Mat arrows = rois[car_nearest_point_index].clone();
        cv::Mat color_arrows;
        cv::cvtColor(arrows, color_arrows, cv::COLOR_GRAY2BGR);

        cv::arrowedLine(color_arrows,
                        goal_in_roi,
                        cv::Point(goal_in_roi.x + 10 * cos(goal_yaw), goal_in_roi.y + 10 * sin(goal_yaw)),
                        cv::Scalar(255, 0, 255), 1);  // BGR color: fucsia (correct)
        cv::arrowedLine(color_arrows,
                        goal_in_roi,
                        cv::Point(goal_in_roi.x + 10 * cos(heading[goal_idx] + M_PI_2),
                                  goal_in_roi.y + 10 * sin(heading[goal_idx] + M_PI_2)),
                        cv::Scalar(255, 0, 0), 1);  // color: blue (wrong)

        cv::arrowedLine(color_arrows,
                        current_car_pose_in_roi,
                        cv::Point(current_car_pose_in_roi.x + 10 * cos(initial_yaw),
                                  current_car_pose_in_roi.y + 10 * sin(initial_yaw)),
                        cv::Scalar(0, 0, 255), 1);  // color: red (correct)
        cv::arrowedLine(color_arrows,
                        current_car_pose_in_roi,
                        cv::Point(current_car_pose_in_roi.x + 10 * cos(tf2::getYaw(current_pose.orientation)),
                                  current_car_pose_in_roi.y + 10 * sin(tf2::getYaw(current_pose.orientation))),
                        cv::Scalar(0, 255, 0), 1);  // color: green (wrong)

        double scale_factor = 3.0; // Change this to your desired scale. >1.0 for upscaling, <1.0 for downscaling.
        cv::resize(color_arrows, color_arrows, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);

        cv::imshow("Arrows", color_arrows);
        cv::waitKey(20);
    }
    
    // Convert the path to meters staying in the ROI frame
    std::list<tuple<float, float>> aap_m;
    for (const auto& coord : aap) {
        float x = resolution * std::get<0>(coord);
        float y = resolution * std::get<1>(coord);
        aap_m.emplace_back(x, y);
    }

    // Replace first and last points with the accurate car and goal positions
    aap_m.front() = {current_car_pose_in_roi_subpx.x * resolution, current_car_pose_in_roi_subpx.y * resolution};
    aap_m.back() = {goal_in_roi_subpx.x * resolution, goal_in_roi_subpx.y * resolution};

    AnyAnglePath::Ptr path = AnyAnglePath::process_theta_star_output(aap_m, initial_yaw, goal_yaw);

    std::vector<float> rrt_path_cx;
    std::vector<float> rrt_path_cy;
    std::vector<float> rrt_path_length;
    std::vector<float> rrt_path_speed;
    bool rrt_trajectory_found;
    get_rrt_trajectory(*path, rrt_path_length, rrt_path_cx, rrt_path_cy, rrt_path_speed, rrt_trajectory_found);
    if (not rrt_trajectory_found) {
        return;
    }

    std::vector<float> cx_new;
    std::vector<float> cy_new;
    std::vector<float> length_new;
    std::vector<float> speed_new;
    int start_index = this->car_nearest_point_index;
    int goal_index = this->goal_idx;

    if (start_index < goal_index) {
        cx_new = rrt_path_cx;
        cx_new.insert(cx_new.end(), this->cx.begin() + this->goal_idx, this->cx.end());
        cx_new.insert(cx_new.end(), this->cx.begin(), this->cx.begin() + this->car_nearest_point_index);

        cy_new = rrt_path_cy;
        cy_new.insert(cy_new.end(), this->cy.begin() + this->goal_idx, this->cy.end());
        cy_new.insert(cy_new.end(), this->cy.begin(), this->cy.begin() + this->car_nearest_point_index);

        float original_length_to_goal = this->length[this->goal_idx];
        float rrt_length = rrt_path_length.back();
        length_new = rrt_path_length;
        for (size_t i = goal_index; i < this->length.size(); i++) {
            length_new.push_back(this->length[i] - original_length_to_goal + rrt_length);
        }
        float length_original_last_idx = length_new.back();
        for (int i = 0; i < this->car_nearest_point_index; i++) {
            length_new.push_back(this->length[i] + length_original_last_idx);
        }

        speed_new = rrt_path_speed;
        speed_new.insert(speed_new.end(), this->speed.begin() + this->goal_idx, this->speed.end());
        speed_new.insert(speed_new.end(), this->speed.begin(), this->speed.begin() + this->car_nearest_point_index);
    } else {
        cx_new = rrt_path_cx;
        cx_new.insert(cx_new.end(), this->cx.begin() + this->goal_idx, this->cx.begin() + this->car_nearest_point_index);

        cy_new = rrt_path_cy;
        cy_new.insert(cy_new.end(), this->cy.begin() + this->goal_idx, this->cy.begin() + this->car_nearest_point_index);

        float original_length_to_goal = this->length[this->goal_idx];
        float rrt_length = rrt_path_length.back();
        length_new = rrt_path_length;
        for (int i = goal_index; i < this->car_nearest_point_index; i++) {
            length_new.push_back(this->length[i] - original_length_to_goal + rrt_length);
        }

        speed_new = rrt_path_speed;
        speed_new.insert(speed_new.end(), this->speed.begin() + this->goal_idx, this->speed.begin() + this->car_nearest_point_index);
    }

    // Publish new raceline
    std_msgs::msg::Header common_header;
    common_header.stamp = this->get_clock()->now();
    common_header.frame_id = "map";

    nav_msgs::msg::Path path_msg;
    path_msg.header = common_header;
    for (size_t i = 0; i < cx_new.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = cx_new[i];
        pose.pose.position.y = cy_new[i];
        path_msg.poses.push_back(pose);
    }

    f1tenth_msgs::msg::Raceline raceline_msg_;
    raceline_msg_.length = length_new;
    raceline_msg_.x = cx_new;
    raceline_msg_.y = cy_new;
    raceline_msg_.speed = speed_new;
    raceline_msg_.start_idx = start_index;
    raceline_msg_.end_idx = goal_index;

    f1tenth_msgs::msg::RacelineStamped raceline_msg;
    raceline_msg.header = common_header;
    raceline_msg.raceline = raceline_msg_;

    raceline_publisher_->publish(raceline_msg);
	path_publisher_->publish(path_msg);

    new_trajectory_end_index = rrt_path_cx.size() - 1;
    new_trajectory_just_sent = true;
    check_to_send_original_raceline = true;
}


bool RRTMainNode::is_point_in_free_space(const cv::Mat& roi, geos_t x, geos_t y, std::string unit, bool verbose) const {
    int px_x, px_y;
    if (unit == "meter") {
        px_x = my_round(x / resolution);
        px_y = my_round(y / resolution);
    } else if (unit == "pixel") {
        px_x = my_round(x);
        px_y = my_round(y);
    } else {
        throw std::runtime_error("Unit not recognized");
    }

    // Ensure the coordinates are within the image dimensions
    if (px_x >= 0 and px_x < roi.cols and px_y >= 0 and px_y < roi.rows) {
        // Access the pixel value
        // NOTE: here we invert the coordinates because the `cv::Mat::at` method wants the rows first
        uchar pixel_value = roi.at<uchar>(px_y, px_x);

        // Check if the pixel is white (255)
        if (pixel_value == 255) {
            return true;
        }
    } else {
        if (verbose) {
            RCLCPP_WARN_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT,
                                 "Coordinates are out of bounds: (x: %d, y: %d)", px_x, px_y);
        }
    }

    // If coordinates are out of bounds or pixel is not white
    return false;
}


bool RRTMainNode::is_point_within_roi(const cv::Mat& roi, geos_t x, geos_t y, std::string unit) const {
	int px_x, px_y;
    if (unit == "meter") {
        px_x = my_round(x / resolution);
        px_y = my_round(y / resolution);
    } else if (unit == "pixel") {
        px_x = my_round(x);
        px_y = my_round(y);
    } else {
        throw std::runtime_error("Unit not recognized");
    }

    // Ensure the coordinates are within the image dimensions
    if (px_x >= 0 and px_x < roi.cols and px_y >= 0 and px_y < roi.rows) {
        return true;
    }

    return false;
}


void convert_output(const std::vector<Geometry::Ptr>& geoms, std::string filename) {
    // Function for debugging purposes
    WKTWriter writer;
    std::ofstream out(filename);

    for (size_t i = 0; i < geoms.size(); i++) {
        std::string wkt = writer.write(geoms[i].get());
        out << wkt << std::endl;
    }

    out.close();
}
