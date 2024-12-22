#include <rrt/rrt_node.hpp>


void RRTMainNode::calculate_roi(int car_nearest_raceline_index) {
    // (0) let's DECIDE an INDEX subset to create the ROI around
    int starting_index = car_nearest_raceline_index - this->RACELINE_POINTS_BEFORE;
    if (starting_index < 0) {
        starting_index += this->num_of_raceline_points;
    }
    int raceline_steps = this->RACELINE_POINTS_BEFORE + this->RACELINE_POINTS_AFTER;

    // (1) let's FIND OUT THETA
    cv::Point2d car_position = cv::Point2d(this->cx[car_nearest_raceline_index], this->cy[car_nearest_raceline_index]);
    cv::Point2d right_point = cv::Point2d(this->right_x[car_nearest_raceline_index], this->right_y[car_nearest_raceline_index]);
    cv::Point2d right_point_in_the_car_rs = right_point - car_position;
    double theta = atan2(right_point_in_the_car_rs.y, right_point_in_the_car_rs.x);

    // (2) let's TAKE the RACING LINE points and their PROJECTIONS on the BORDER to get a RECTANGLE in an image ROTATED respect to the ORIGINAL ONE
    std::vector<cv::Point2d> points_to_consider;
    for (int i = 0; i < raceline_steps; i++) {
        int an_index = starting_index + i;
        if (an_index >= this->num_of_raceline_points) {
            an_index -= this->num_of_raceline_points;
        }
        points_to_consider.push_back(this->rotate_a_point(cv::Point2d(this->left_x[an_index], this->left_y[an_index]), car_position, -theta));
        points_to_consider.push_back(this->rotate_a_point(cv::Point2d(this->right_x[an_index], this->right_y[an_index]), car_position, -theta));
    }

    double max_x = points_to_consider[0].x;
    double min_x = points_to_consider[0].x;
    double max_y = points_to_consider[0].y;
    double min_y = points_to_consider[0].y;

    for (int i = 0; i < (int)points_to_consider.size(); i++) {
        if (points_to_consider[i].x > max_x) {
            max_x = points_to_consider[i].x;
        }
        if (points_to_consider[i].x < min_x) {
            min_x = points_to_consider[i].x;
        }
        if (points_to_consider[i].y > max_y) {
            max_y = points_to_consider[i].y;
        }
        if (points_to_consider[i].y < min_y) {
            min_y = points_to_consider[i].y;
        }
    }

    // (3) let's ROTATE BACK the obtained 4 VERTEX of the SELECTED RECTANGLE
    std::vector<cv::Point2d> rectangle_m;
    rectangle_m.push_back(this->rotate_a_point(cv::Point2d(min_x, min_y), cv::Point2d(0, 0), theta));
    rectangle_m.push_back(this->rotate_a_point(cv::Point2d(min_x, max_y), cv::Point2d(0, 0), theta));
    rectangle_m.push_back(this->rotate_a_point(cv::Point2d(max_x, max_y), cv::Point2d(0, 0), theta));
    rectangle_m.push_back(this->rotate_a_point(cv::Point2d(max_x, min_y), cv::Point2d(0, 0), theta));

    // (4) let's pass from METERS coordinate system to the PIXELS one
    std::vector<cv::Point> windowPoints;
    for (int i = 0; i < (int)rectangle_m.size(); i++) {
        rectangle_m[i] += car_position;
        windowPoints.push_back(cv::Point(this->m_to_px_x(rectangle_m[i].x), this->m_to_px_y(rectangle_m[i].y)));
    }

    // (5) let's CREATE the ROI_ON_MAP IMAGE
    cv::Mat roi_on_map_image;
    cv::cvtColor(this->map_image.clone(), roi_on_map_image, cv::COLOR_GRAY2BGR);
    int car_x = this->m_to_px_x(car_position.x);
    int car_y = this->m_to_px_y(car_position.y);
    cv::Point center(car_x, car_y);
    int radius = 3;
    cv::polylines(roi_on_map_image,
                  windowPoints,
                  true,
                  cv::Scalar(255, 0, 0),
                  2);
    cv::circle(roi_on_map_image,
               center,
               radius,
               cv::Scalar(0, 255, 0),
               -1);
    cv::putText(roi_on_map_image, "nearest point on racing line", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
    cv::putText(roi_on_map_image, "ROI borders", cv::Point(20, 55), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 0, 0), 2);
    this->rois_on_map[car_nearest_raceline_index] = roi_on_map_image;

    // (6) let's CREATE the ROI_IMAGE
    cv::Mat map_image_clone = this->map_image.clone();
    /*
    cv::circle( map_image_clone,
                center,
                radius,
                cv::Scalar(0, 0, 255),
                -1);
    */

    int shift_x = -car_x + (int)(this->map_image.cols / 2);
    int shift_y = -car_y + (int)(this->map_image.rows / 2);

    cv::Mat translation_matrix = (cv::Mat_<double>(2, 3) << 1, 0, shift_x, 0, 1, shift_y);

    cv::Mat shifted_image;
    cv::warpAffine(map_image_clone, shifted_image, translation_matrix, map_image_clone.size());
    cv::Mat inverted_translation_matrix;
    cv::invertAffineTransform(translation_matrix, inverted_translation_matrix);

    cv::Point2f rotation_center(    (int)(this->map_image.cols / 2),
                                    (int)(this->map_image.rows / 2)    );
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(rotation_center, - theta / M_PI * 180, 1.0);
    cv::Mat rotated_image;
    cv::warpAffine(shifted_image, rotated_image, rotation_matrix, map_image_clone.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::Mat inverted_rotation_matrix;
    cv::invertAffineTransform(rotation_matrix, inverted_rotation_matrix);

    // Let's translate the rectangle points
    std::vector<cv::Point2f> vec;
    for (int i = 0; i < 4; i++) {
        cv::Point2f a_point;
        a_point.x = (float)this->m_to_px_x(rectangle_m[i].x);
        a_point.y = (float)this->m_to_px_y(rectangle_m[i].y);
        vec.push_back(a_point);
    }
    cv::transform(vec, vec, translation_matrix);
    cv::transform(vec, vec, rotation_matrix);

    cv::Rect roi(vec[1].x, vec[1].y, abs(vec[0].x - vec[3].x), abs(vec[1].y - vec[0].y));
    cv::Mat final_roi = rotated_image(roi);

    // (7) let's DILATE the ROI
    cv::Mat element = cv::getStructuringElement(
        this->dilation_kernel_shape,
        cv::Size    (   2*this->TRACK_DILATION_SIZE + 1,    2*this->TRACK_DILATION_SIZE+1   ),
        cv::Point   (   this->TRACK_DILATION_SIZE,          this->TRACK_DILATION_SIZE       )
        );
    cv::erode(final_roi, final_roi, element);

    // Let's make the ROI binary (i.e., 0 or 255). Remove interpolation artifacts
    for (int i = 0; i < final_roi.rows; i++) {
        for (int j = 0; j < final_roi.cols; j++) {
            if (final_roi.at<uchar>(i, j) < 255) {
                final_roi.at<uchar>(i, j) = 0;
            }
        }
    }

    for (int i = 0; i < final_roi.rows; i++) {
        for (int j = 0; j < final_roi.cols; j++) {
            assert(final_roi.at<uchar>(i, j) == 0 or final_roi.at<uchar>(i, j) == 255);
        }
    }

    this->rotation_angles                [car_nearest_raceline_index] = theta;
    this->translation_matrices           [car_nearest_raceline_index] = translation_matrix;
    this->inverse_translation_matrices   [car_nearest_raceline_index] = inverted_translation_matrix;
    this->rotation_matrices              [car_nearest_raceline_index] = rotation_matrix;
    this->inverse_rotation_matrices      [car_nearest_raceline_index] = inverted_rotation_matrix;
    this->values_to_subtract             [car_nearest_raceline_index] = vec[1];

    cv::Point2f raceline_point_in_roi = this->from_world_to_roi_subpx_with_index(cv::Point2f(
            this->cx[car_nearest_raceline_index], this->cy[car_nearest_raceline_index]), car_nearest_raceline_index);
    if (final_roi.at<uchar>(my_round(raceline_point_in_roi.y), my_round(raceline_point_in_roi.x)) != 255) {
        RCLCPP_ERROR(get_logger(), "Raceline point in ROI: (%.4f, %.4f)", raceline_point_in_roi.x, raceline_point_in_roi.y);
        RCLCPP_ERROR(get_logger(), "final_roi.at<uchar>(raceline_point_in_roi.y, raceline_point_in_roi.x) = %d",
                     final_roi.at<uchar>(raceline_point_in_roi.y, raceline_point_in_roi.x));
        RCLCPP_ERROR(get_logger(), "car_nearest_raceline_index = %d", car_nearest_raceline_index);
        RCLCPP_ERROR(get_logger(), "This raceline point is not in the ROI free space! Consider decreasing the "
                                   "TRACK_DILATION_SIZE parameter or generating a new raceline.");
        RCLCPP_WARN(get_logger(), "If VISUALIZE_ROI is true, you can see the point in the ROI space in the desktop "
                                  "environment.");

        if (VISUALIZE_ROI) {
            cv::Mat tmp;
            cv::cvtColor(final_roi.clone(), tmp, cv::COLOR_GRAY2BGR);
            cv::Point center(raceline_point_in_roi.x, raceline_point_in_roi.y);
            int radius = 3;
            cv::circle(tmp,
                       center,
                       radius,
                       cv::Scalar(0, 0, 255),
                       -1);
            cv::resize(tmp, tmp, cv::Size(), 3.0, 3.0, cv::INTER_NEAREST);
            cv::imshow("ROI error - raceline point on obstacle space [rrt]", tmp);
            cv::waitKey(20);
        }

        throw std::runtime_error("Raceline point is not in the ROI free space!");
    }

    // (8) Blacken other white regions in roi
    cv::Point point_in_roi(my_round(raceline_point_in_roi.x), my_round(raceline_point_in_roi.y));

    // Ensure the image is binary
    cv::Mat binary_roi;
    cv::threshold(final_roi, binary_roi, 127, 255, cv::THRESH_BINARY);

    // Get connected components
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(binary_roi, labels, stats, centroids);

    // Find the label of the component that includes the point
    int target_label = labels.at<int>(point_in_roi);

    // Create a new image where only the pixels of the found component are white
    cv::Mat component_image = cv::Mat::zeros(binary_roi.size(), CV_8U);
    for (int y = 0; y < binary_roi.rows; y++) {
        for (int x = 0; x < binary_roi.cols; x++) {
            if (labels.at<int>(y,x) == target_label) {
                component_image.at<uchar>(y,x) = 255;
            }
        }
    }

    this->rois[car_nearest_raceline_index] = component_image;

    // (9) create polygon from connected component
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> contour;
    cv::findContours(component_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    assert(contours.size() == 1);
    contour = contours[0];

    CoordinateSequence coordinates(/*size=*/0, /*dims=*/2);

    // INFO: should we invert coords here? No. It has to be the same coordinate system as the AnyAnglePath object one
    for (const auto& point : contour) { // Assuming the largest contour is the one of interest
        coordinates.add(resolution * point.x, resolution * point.y);
    }
    // Ensure the ring is closed by adding the first point at the end
    coordinates.add(coordinates[0]);

    std::unique_ptr<LinearRing> linear_ring = factory->createLinearRing(coordinates);
    assert(linear_ring->isClosed());

    std::unique_ptr<Polygon> polygon_ = factory->createPolygon(std::move(linear_ring));
    Geometry::Ptr polygon = DouglasPeuckerSimplifier::simplify(polygon_.get(), 0.005);
    assert(polygon->isValid());

    // Add the polygon to the list of polygons
    this->rois_free_space_polygons[car_nearest_raceline_index] = std::move(polygon);
}


cv::Point2f RRTMainNode::from_world_to_roi(cv::Point2f a_point) const {
    // Don't use this function unless you know what you are doing

    a_point.x = (float)this->m_to_px_x(a_point.x);
    a_point.y = (float)this->m_to_px_y(a_point.y);
    std::vector<cv::Point2f> vec;
    vec.push_back(a_point);
    cv::transform(vec, vec, this->translation_matrices[this->car_nearest_point_index]);
    cv::transform(vec, vec, this->rotation_matrices[this->car_nearest_point_index]);
    vec[0].x -= this->values_to_subtract[this->car_nearest_point_index].x;
    vec[0].y -= this->values_to_subtract[this->car_nearest_point_index].y;
    return vec[0];
}


cv::Point2f RRTMainNode::from_world_to_roi_with_index(cv::Point2f a_point, int car_nearest_point_index) const {
    // Don't use this function unless you know what you are doing

    a_point.x = (float)this->m_to_px_x(a_point.x);
    a_point.y = (float)this->m_to_px_y(a_point.y);
    std::vector<cv::Point2f> vec;
    vec.push_back(a_point);
    cv::transform(vec, vec, this->translation_matrices[car_nearest_point_index]);
    cv::transform(vec, vec, this->rotation_matrices[car_nearest_point_index]);
    vec[0].x -= this->values_to_subtract[car_nearest_point_index].x;
    vec[0].y -= this->values_to_subtract[car_nearest_point_index].y;
    return vec[0];
}


cv::Point2f RRTMainNode::from_world_to_roi_subpx_with_index(cv::Point2f a_point, int car_nearest_point_index) const {
    // Don't use this function unless you know what you are doing

    a_point.x = this->m_to_subpx_x(a_point.x);
    a_point.y = this->m_to_subpx_y(a_point.y);
    std::vector<cv::Point2f> vec;
    vec.push_back(a_point);
    cv::transform(vec, vec, this->translation_matrices[car_nearest_point_index]);
    cv::transform(vec, vec, this->rotation_matrices[car_nearest_point_index]);
    vec[0].x -= this->values_to_subtract[car_nearest_point_index].x;
    vec[0].y -= this->values_to_subtract[car_nearest_point_index].y;
    return vec[0];
}


cv::Point2f RRTMainNode::from_roi_to_world(cv::Point2f a_point) const {
    // Don't use this function unless you know what you are doing

    a_point.x += this->values_to_subtract[this->car_nearest_point_index].x;
    a_point.y += this->values_to_subtract[this->car_nearest_point_index].y;
    std::vector<cv::Point2f> vec;
    vec.push_back(a_point);
    cv::transform(vec, vec, this->inverse_rotation_matrices[this->car_nearest_point_index]);
    cv::transform(vec, vec, this->inverse_translation_matrices[this->car_nearest_point_index]);
    vec[0].x = (float)this->px_to_m_x(vec[0].x);
    vec[0].y = (float)this->px_to_m_y(vec[0].y);
    return vec[0];
}


void RRTMainNode::from_roi_to_world(const std::vector<float>& x_roi,
                                    const std::vector<float>& y_roi,
                                    std::vector<float>& x_world,
                                    std::vector<float>& y_world) const {
    // Don't use this function unless you know what you are doing

    std::vector<cv::Point2f> vec;
    for (size_t i = 0; i < x_roi.size(); i++) {
        vec.emplace_back(x_roi[i] + values_to_subtract[car_nearest_point_index].x,
                         y_roi[i] + values_to_subtract[car_nearest_point_index].y);
    }

    cv::transform(vec, vec, inverse_rotation_matrices[car_nearest_point_index]);
    cv::transform(vec, vec, inverse_translation_matrices[car_nearest_point_index]);

    x_world.resize(x_roi.size());
    y_world.resize(y_roi.size());
    for (size_t i = 0; i < x_roi.size(); i++) {
        x_world[i] = subpx_to_m_x(vec[i].x);
        y_world[i] = subpx_to_m_y(vec[i].y);
    }
}
