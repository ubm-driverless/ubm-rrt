#include <rrt/rrt_node.hpp>


void RRTMainNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    auto start_time = std::chrono::steady_clock::now();

    if (not raceline_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for raceline...");
    }
    if (not pose_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for pose...");
    }
    if (not look_ahead_point_index_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for look ahead point index... (Is the path-following algo running?)");
    }
    if (not car_nearest_point_index_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for car nearest point index... (Is the path-following algo running?)");
    }

    if (not raceline_obtained or not pose_obtained or not look_ahead_point_index_obtained or not car_nearest_point_index_obtained) {
        return;
    }

    // Check that current car pose in inside free ROI region and within ROI bounds (using `is_point_in_free_space`)
    current_car_pose_in_roi_subpx = from_world_to_roi_subpx_with_index(cv::Point2f(real_car_x, real_car_y), car_nearest_point_index);
    current_car_pose_in_roi = cv::Point(my_round(current_car_pose_in_roi_subpx.x), my_round(current_car_pose_in_roi_subpx.y));
    if (not is_point_in_free_space(rois[car_nearest_point_index], current_car_pose_in_roi.x, current_car_pose_in_roi.y, "pixel")) {
        RCLCPP_WARN_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT,
                             "Car POSE not in free space - ROI index: %d - ROI shape: %d, %d",
                             car_nearest_point_index, rois[car_nearest_point_index].rows,
                             rois[car_nearest_point_index].cols);
        return;
    }

    goal_idx = (car_nearest_point_index + RACELINE_POINTS_AFTER - RRT_GOAL_OFFSET) % num_of_raceline_points;
    if (car_nearest_point_index == goal_idx) {
        RCLCPP_WARN(get_logger(), "Car nearest point index is the same as goal index");
        return;
    }
    goal_in_roi_subpx = from_world_to_roi_subpx_with_index(cv::Point2f(cx[goal_idx], cy[goal_idx]), car_nearest_point_index);
    goal_in_roi = cv::Point(my_round(goal_in_roi_subpx.x), my_round(goal_in_roi_subpx.y));
    if (not is_point_in_free_space(rois[car_nearest_point_index], goal_in_roi.x, goal_in_roi.y, "pixel")) {
        RCLCPP_WARN_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT,
                             "GOAL not in free space - ROI index: %d - ROI shape: %d, %d", car_nearest_point_index,
                             rois[car_nearest_point_index].rows, rois[car_nearest_point_index].cols);
        return;
    }

    current_roi = rois[car_nearest_point_index].clone();
    std::vector<cv::Point2f> lidar_points_in_roi_tmp;
    if (VISUALIZE_ROI) {
        cv::cvtColor(current_roi.clone(), visualized_roi, cv::COLOR_GRAY2BGR);
    }

    current_free_space = rois_free_space_polygons[car_nearest_point_index]->clone();

    std::unique_ptr<Polygon> dilated_lidar_meters;

    // Dilate every lidar point and update the ROI and free space
    // IDEA: process only lidar points that do not belong to the track borders. This would allow to dilate only the
    //       actual obstacles
    for (size_t i = 0; i < scan_msg->ranges.size(); i += (1 + SCAN_STEP)) {
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        float range = scan_msg->ranges[i];

        if (range > MAX_LIDAR_RANGE) {
            continue;
        }

        float lidar_point_x = real_car_x + range * std::cos(angle + current_yaw);
        float lidar_point_y = real_car_y + range * std::sin(angle + current_yaw);

        cv::Point2f lidar_point_in_roi = from_world_to_roi_subpx_with_index(cv::Point2f(lidar_point_x, lidar_point_y), car_nearest_point_index);

        if (not is_point_within_roi(rois[car_nearest_point_index], lidar_point_in_roi.x, lidar_point_in_roi.y, "pixel")) {
            continue;
        }

        CoordinateXY lidar_coord(lidar_point_in_roi.x, lidar_point_in_roi.y);
        Point::Ptr lidar_point = factory->createPoint(lidar_coord);

        Geometry::Ptr dilated_lidar_point = lidar_point->buffer(LIDAR_OBSTACLE_DILATION_SIZE / resolution, 8);
        CoordinateSequence free_space_dilated_coords = *dilated_lidar_point->getCoordinates();
        std::vector<cv::Point> roi_dilated_coords;
        for (size_t i = 0; i < free_space_dilated_coords.size(); i++) {
            roi_dilated_coords.push_back(cv::Point(free_space_dilated_coords[i].x, free_space_dilated_coords[i].y));
            free_space_dilated_coords.setAt<CoordinateXY>(CoordinateXY(free_space_dilated_coords[i].x * resolution,
                                                                       free_space_dilated_coords[i].y * resolution), i);
        }

        cv::fillConvexPoly(current_roi, roi_dilated_coords, cv::Scalar(0), cv::LINE_8, 0);
        std::unique_ptr<LinearRing> dilated_lidar_ring_meters = factory->createLinearRing(free_space_dilated_coords);
        dilated_lidar_meters = factory->createPolygon(std::move(dilated_lidar_ring_meters));

        current_free_space = current_free_space->difference(dilated_lidar_meters.get());

        if (VISUALIZE_ROI) {
            cv::fillConvexPoly(visualized_roi, roi_dilated_coords, cv::Scalar(0), cv::LINE_8, 0);
            lidar_points_in_roi_tmp.push_back(lidar_point_in_roi);
        }
    }

    if (VISUALIZE_ROI) {
        for (const auto& lidar_point_in_roi : lidar_points_in_roi_tmp) {
            int radius = 1;
            cv::circle(visualized_roi,
                       lidar_point_in_roi,
                       radius,
                       cv::Scalar(255, 0, 0),
                       -1);
        }

        cv::circle(visualized_roi,
                   current_car_pose_in_roi_subpx,
                   1,
                   cv::Scalar(0, 0, 255),
                   -1);
        cv::circle(visualized_roi,
                   goal_in_roi_subpx,
                   1,
                   cv::Scalar(0, 255, 0),
                   -1);
    }

    // Create a string timestamp
    // std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    // std::vector<Geometry::Ptr> geoms;
    // geoms.push_back(std::move(current_free_space));
    // convert_output(geoms, "/home/ubm/repo/rrt_cpp/tests/dilated_lidar_" + timestamp + ".wkt");
    // throw std::runtime_error("Dilated lidar points saved");

    this->rrt_main();

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_DEBUG(get_logger(), "Elapsed time: %d ms", elapsed_time.count());
}


void RRTMainNode::obstacle_callback(const f1tenth_msgs::msg::ObstacleArray::SharedPtr msg) {
    RCLCPP_WARN(get_logger(), "Obstacle callback called");

    auto start_time = std::chrono::steady_clock::now();

    if (not raceline_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for raceline...");
    }
    if (not pose_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for pose...");
    }
    if (not look_ahead_point_index_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for look ahead point index... (Is the path-following algo running?)");
    }
    if (not car_nearest_point_index_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for car nearest point index... (Is the path-following algo running?)");
    }

    if (not raceline_obtained or not pose_obtained or not look_ahead_point_index_obtained or not car_nearest_point_index_obtained) {
        return;
    }

    const std::vector<f1tenth_msgs::msg::Obstacle>& obstacles = msg->obstacles;
    if (obstacles.size() == 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "No obstacles detected. Skipping RRT");
        return;
    }

    // Check that current car pose in inside free ROI region and within ROI bounds (using `is_point_in_free_space`)
    cv::Point2f current_car_pose_in_roi_ = from_world_to_roi_subpx_with_index(cv::Point2f(real_car_x, real_car_y),
                                                                              car_nearest_point_index);
    current_car_pose_in_roi = cv::Point(my_round(current_car_pose_in_roi_.x), my_round(current_car_pose_in_roi_.y));
    if (not is_point_in_free_space(rois[car_nearest_point_index], current_car_pose_in_roi.x, current_car_pose_in_roi.y,
                                   "pixel")) {
        RCLCPP_WARN_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT,
                             "Car POSE not in free space - ROI index: %d - ROI shape: %d, %d",
                             car_nearest_point_index, rois[car_nearest_point_index].rows,
                             rois[car_nearest_point_index].cols);
        return;
    }

    goal_idx = (car_nearest_point_index + RACELINE_POINTS_AFTER - RRT_GOAL_OFFSET) % num_of_raceline_points;
    if (car_nearest_point_index == goal_idx) {
        RCLCPP_WARN(get_logger(), "Car nearest point index is the same as goal index");
        return;
    }
    cv::Point2f goal_in_roi_ = from_world_to_roi_subpx_with_index(cv::Point2f(cx[goal_idx], cy[goal_idx]), car_nearest_point_index);
    goal_in_roi = cv::Point(my_round(goal_in_roi_.x), my_round(goal_in_roi_.y));
    if (not is_point_in_free_space(rois[car_nearest_point_index], goal_in_roi.x, goal_in_roi.y, "pixel")) {
        RCLCPP_WARN_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT,
                             "GOAL not in free space - ROI index: %d - ROI shape: %d, %d", car_nearest_point_index,
                             rois[car_nearest_point_index].rows, rois[car_nearest_point_index].cols);
        return;
    }

    bool obstacle_obtained = false;
    current_roi = rois[car_nearest_point_index].clone();
    current_free_space = rois_free_space_polygons[car_nearest_point_index]->clone();

    // For each obstacle...
    for (size_t i = 0; i < obstacles.size(); i++) {
        const f1tenth_msgs::msg::Obstacle& obs = obstacles[i];

        // Get rectangle vertices
        cv::Point2f point1_map(obs.point_1[0], obs.point_1[1]);
        cv::Point2f point2_map(obs.point_2[0], obs.point_2[1]);
        cv::Point2f point3_map(obs.point_3[0], obs.point_3[1]);
        cv::Point2f point4_map(obs.point_4[0], obs.point_4[1]);

        // Convert obstacles points to roi frame
        cv::Point2f point1_roi = from_world_to_roi(point1_map);
        cv::Point2f point2_roi = from_world_to_roi(point2_map);
        cv::Point2f point3_roi = from_world_to_roi(point3_map);
        cv::Point2f point4_roi = from_world_to_roi(point4_map);

        point1_roi = cv::Point(my_round(point1_roi.x), my_round(point1_roi.y));
        point2_roi = cv::Point(my_round(point2_roi.x), my_round(point2_roi.y));
        point3_roi = cv::Point(my_round(point3_roi.x), my_round(point3_roi.y));
        point4_roi = cv::Point(my_round(point4_roi.x), my_round(point4_roi.y));

        CoordinateSequence coordinates(/*size=*/0, /*dims=*/2);
        coordinates.add(CoordinateXY(point1_roi.x, point1_roi.y));
        coordinates.add(CoordinateXY(point2_roi.x, point2_roi.y));
        coordinates.add(CoordinateXY(point3_roi.x, point3_roi.y));
        coordinates.add(CoordinateXY(point4_roi.x, point4_roi.y));
        coordinates.add(CoordinateXY(point1_roi.x, point1_roi.y));
        std::unique_ptr<CoordinateSequence> coordinates_ptr = std::make_unique<CoordinateSequence>(coordinates);

        // Check if the obstacle is valid
        std::unique_ptr<LinearRing> ring;
        try {
            ring = std::make_unique<LinearRing>(std::move(coordinates_ptr), *factory);
        } catch (const geos::util::IllegalArgumentException& e) {
            RCLCPP_WARN(get_logger(), "Invalid obstacle detected. The vertices of the obstacles were not ordered correctly");
            continue;
        }
        assert(ring->isClosed());

        // Dilate the obstacle
        std::unique_ptr<Polygon> polygon = factory->createPolygon(std::move(ring));
        assert(polygon->isValid());
        Geometry::Ptr dilated_polygon = polygon->buffer(OBSTACLE_DILATION_SIZE, 4);

        // Check that current car pose in inside free ROI region after dilation (if that's the case, send a slow-down signal and skip RRT)
        CoordinateXY car_pose_coord(current_car_pose_in_roi_.x, current_car_pose_in_roi_.y);
        Point::Ptr car_pose_point = factory->createPoint(car_pose_coord);
        if (car_pose_point->within(dilated_polygon.get())) {
            RCLCPP_WARN(get_logger(), "Car pose inside dilated obstacle. Skipping RRT");
            if (send_slow_down) {
                RCLCPP_ERROR(get_logger(), "Sending slow-down signal");
                std_msgs::msg::Bool slow_down_msg;
                slow_down_msg.data = true;
                slow_down_publisher_->publish(slow_down_msg);
            }
            return;
        }

        // Check if the car is too close to a valid obstacle (if that's the case, send a slow-down signal and skip RRT)
        if (check_min_distance_to_obstacle) {
            double pixel_distance = car_pose_point->distance(dilated_polygon.get());
            if (pixel_distance * resolution < MIN_DISTANCE_TO_OBSTACLE) {
                RCLCPP_WARN(get_logger(), "Car too close to obstacle. Skipping RRT");
                if (send_slow_down) {
                    RCLCPP_ERROR(get_logger(), "Sending slow-down signal");
                    std_msgs::msg::Bool slow_down_msg;
                    slow_down_msg.data = true;
                    slow_down_publisher_->publish(slow_down_msg);
                }
                return;
            }
        }

        // TODO: maybe is better to consider the distance instead of the intersection
        // Check if the obstacle intersects with the raceline (if not, skip RRT)
        CoordinateSequence raceline_coords(/*size=*/0, /*dims=*/2);
        for (int j = 0; j < RACELINE_POINTS_AFTER; j++) {
            int index = (car_nearest_point_index + j) % num_of_raceline_points;
            cv::Point2f point_map(cx[index], cy[index]);
            cv::Point2f point_roi = from_world_to_roi(point_map);
            raceline_coords.add(CoordinateXY(point_roi.x, point_roi.y));
        }
        std::unique_ptr<LineString> raceline_ls = factory->createLineString(raceline_coords);
        if (not raceline_ls->intersects(dilated_polygon.get())) {
            RCLCPP_WARN(get_logger(), "Obstacle does not intersect with raceline. Skipping RRT");
            return;
        }

        // Update ROI with the dilated obstacle
        std::vector<cv::Point> polygon_v;
        polygon_v.push_back(point1_roi);
        polygon_v.push_back(point2_roi);
        polygon_v.push_back(point3_roi);
        polygon_v.push_back(point4_roi);
        cv::fillConvexPoly(current_roi, polygon_v, cv::Scalar(0), cv::LINE_8, 0);

        // Update free space with the dilated obstacle
        CoordinateSequence poly_coords = *dilated_polygon->getCoordinates();
        for (size_t j = 0; j < poly_coords.size(); j++) {
            poly_coords.setAt<CoordinateXY>(CoordinateXY(poly_coords[j].x * resolution,
                                                         poly_coords[j].y * resolution), j);
        }
        std::unique_ptr<LinearRing> dilated_poly_ring_meters = factory->createLinearRing(poly_coords);
        std::unique_ptr<Polygon> dilated_poly_meters = factory->createPolygon(std::move(dilated_poly_ring_meters));

        current_free_space = current_free_space->difference(dilated_poly_meters.get());
        // TODO: Create a prepared polygon for the free space

        obstacle_obtained = true;
    }

    if (obstacle_obtained) {
        this->rrt_main();
    }

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_DEBUG(get_logger(), "Elapsed time: %d ms", elapsed_time.count());
}
