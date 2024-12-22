#include <rrt/rrt_node.hpp>
#include <rrt/speed_profile.hpp>


float GP::strip_width = 1.0;
int GP::n_iterations = 50;
geos_t GP::eps_L2_squared = 0.01;   // TODO: eps_L2_squared must be greater than k_gamma
geos_t GP::delta_R_squared = 2.0;
int GP::max_neighbors = 10;
geos_t GP::eps_L2_eq = 1e-30;
double GP::simplify_tolerance = 0.01;
int GP::buffer_window = 2;
int GP::mean_yaw_window = 1;
int GP::buffer_retries = 3;
bool GP::use_rhombus_function = true;
float GP::trapezoidal_one_fraction = 0.8;
bool GP::use_orientation_bias = false;
float GP::orientation_bias = 0.1;
bool GP::enable_X_free_sampling = true;
int GP::X_free_interval_over_nodes = 10;
int GP::X_free_interval_over_iterations = 8;
float GP::goal_region_radius = 0.3;
float GP::w_e = 0.1;
float GP::w_theta = 0.9;
float GP::w_d = 0.2;
float GP::w_q = 0.8;

float GP::dt = 0.05;
float GP::k_alpha = 5.0;
float GP::k_phi = -2.0;
float GP::k_v = 3.8;
float GP::k_gamma = 0.01;
float GP::wb = 0.321;
float GP::speed = 1;
float GP::steer_limit = 0.38;
bool GP::enable_angular_velocity_limit = true;
float GP::k_rho = 1.5;
float GP::k_max_num_iterations = 1.5;
float GP::yaw_tolerance = 0.1;
int GP::downsample_factor = 1;
// w_max(): 1.24

float GP::drag_coeff = 0.2;
float GP::m_veh = 3.158;
float GP::v_max = 11.0;
float GP::dyn_model_exp = 1.0;
float GP::speed_profile_minimum_start_speed = 1.0;


RRTMainNode::RRTMainNode() :
    Node("rrt_node"),
    USE_OBSTACLE_CALLBACK(declare_parameter("use_obstacle_callback", true)),
    MAP_FILEPATH(declare_parameter("map_filepath", "")),
    MAP_YAMLPATH(declare_parameter("map_yamlpath", "")),
    VISUALIZE_ROI(declare_parameter("visualize_roi", true)),
    VISUALIZE_ROI_ON_MAP(declare_parameter("visualize_roi_on_map", true)),
    RACELINE_POINTS_BEFORE(declare_parameter("raceline_points_before", 15)),
    RACELINE_POINTS_AFTER(declare_parameter("raceline_points_after", 15)),
    RRT_GOAL_OFFSET(declare_parameter("rrt_goal_offset", 1)),
    TRACK_DILATION_SHAPE(declare_parameter("track_dilation_shape", 0)),
    TRACK_DILATION_SIZE(declare_parameter("track_dilation_size", 5)),
    OBSTACLE_DILATION_SIZE(declare_parameter("obstacle_dilation_size", 1.0)),
    LIDAR_OBSTACLE_DILATION_SIZE(declare_parameter("lidar_obstacle_dilation_size", 0.1)),
    MAX_LIDAR_RANGE(declare_parameter("max_lidar_range", 2.0)),
    SCAN_STEP(declare_parameter("scan_step", 1)),
    MIN_DISTANCE_TO_OBSTACLE(declare_parameter("min_distance_to_obstacle", 0.2)),
    TOPIC_FOR_REAL_POSITION(declare_parameter("topic_for_real_position", "")),
    SCAN_TOPIC(declare_parameter("scan_topic", "/scan")),
    DISABLE_PRINT_TIMEOUT(declare_parameter("disable_print_timeout", 2000))
{
    // TODO: add all important checks
    if (RACELINE_POINTS_AFTER - RRT_GOAL_OFFSET <= 0) {
        RCLCPP_ERROR(get_logger(), "RACELINE_POINTS_AFTER must be greater than RRT_GOAL_OFFSET");
        throw std::runtime_error("RACELINE_POINTS_AFTER must be greater than RRT_GOAL_OFFSET");
    }

    // TODO: get global parameters from the launch file
    GP::assert_params();

    check_min_distance_to_obstacle = declare_parameter("check_min_distance_to_obstacle", false);
    slow_down_topic = declare_parameter("slow_down_topic", "/rrt/slow_down");
    send_slow_down = declare_parameter("send_slow_down", false);

    ax_max_machines_path = declare_parameter("ax_max_machines_path", "/home/ubm/repo/raceline/raceline/TUM/inputs/veh_dyn_info/ax_max_machines.csv");
    ggv_path = declare_parameter("ggv_path", "/home/ubm/repo/raceline/raceline/TUM/inputs/veh_dyn_info/ggv.csv");
    ax_max_machines = csv_to_mat(ax_max_machines_path);
    ggv = csv_to_mat(ggv_path);

    clock = get_clock();
    if (USE_OBSTACLE_CALLBACK) {
        obstacle_subscriber_ = this->create_subscription<f1tenth_msgs::msg::ObstacleArray>(
            "/obstacles", 1, std::bind(&RRTMainNode::obstacle_callback, this, std::placeholders::_1));
    } else {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            SCAN_TOPIC, 1, std::bind(&RRTMainNode::scan_callback, this, std::placeholders::_1));
    }
    look_ahead_point_index_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
        "/look_ahead_point_index", 1, std::bind(&RRTMainNode::look_ahead_point_index_callback, this, std::placeholders::_1));
    pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        TOPIC_FOR_REAL_POSITION, 1, std::bind(&RRTMainNode::pose_callback, this, std::placeholders::_1));

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos = qos.transient_local().reliable();

    raceline_subscriber_ = this->create_subscription<f1tenth_msgs::msg::RacelineStamped>(
        "/raceline", qos, std::bind(&RRTMainNode::raceline_callback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/odom_point", qos);

    raceline_publisher_ = this->create_publisher<f1tenth_msgs::msg::RacelineStamped>("/raceline", qos);

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path_rrt", qos);

    slow_down_publisher_ = this->create_publisher<std_msgs::msg::Bool>(slow_down_topic, qos);

    // Load Map from MAP_FILEPATH
    this->original_map_image = cv::imread(MAP_FILEPATH, cv::IMREAD_GRAYSCALE);
    if (this->original_map_image.empty()) {
        RCLCPP_ERROR(get_logger(), "Image could not be loaded. Maybe file not found.");
        return;
    }

    // Load Yaml from MAP_YAMLPATH
    std::fstream newfile;
    newfile.open(MAP_YAMLPATH, std::ios::in);
    if (newfile.is_open()) {
        std::string tp;
        while(getline(newfile, tp)){
            // for (int i = 0; i < tp.length(); i++){
            // }
            std::stringstream s(tp);
            std::string attribute_name;
            s >> attribute_name;
            if (attribute_name.compare("resolution:") == 0) {
                std::string resolution_str;
                s >> resolution_str;
                this->resolution = std::stod(resolution_str);
            } else if (attribute_name.compare("origin:") == 0) {
                std::string word_1;
                std::string word_2;
                s >> word_1;
                s >> word_2;
                word_1 = word_1.substr(1, word_1.length()-2);
                word_2 = word_2.substr(0, word_2.length()-1);
                this->o_x = std::stod(word_1);
                this->o_y = std::stod(word_2);
            }
        }
        newfile.close();
    } else {
        RCLCPP_ERROR(get_logger(), "Yaml could not be loaded. Maybe file not found.");
        return;
    }

    if (this->TRACK_DILATION_SHAPE == 1) {
        this->dilation_kernel_shape = cv::MORPH_CROSS;
    } else if (this->TRACK_DILATION_SHAPE == 2) {
        this->dilation_kernel_shape = cv::MORPH_ELLIPSE;
    } else {
        this->dilation_kernel_shape = cv::MORPH_RECT;
    }

    RCLCPP_INFO(get_logger(), "RRTMainNode has been initialized");
}


void RRTMainNode::pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg) {
    if (not raceline_obtained) {
        RCLCPP_INFO_THROTTLE(get_logger(), *clock, DISABLE_PRINT_TIMEOUT, "Waiting for raceline...");
        return;
    }

    this->current_pose = pose_msg->pose.pose;
    this->pose_obtained = true;
    this->real_car_x = this->current_pose.position.x;
    this->real_car_y = this->current_pose.position.y;
    this->current_yaw = tf2::getYaw(pose_msg->pose.pose.orientation);
    this->current_speed = pose_msg->twist.twist.linear.x;

    // Create vector of distances between the car's current position and path
    std::vector<float> dx = std::vector<float>();
    std::vector<float> dy = std::vector<float>();
    std::vector<float> d = std::vector<float>();
    for (unsigned int i = 0; i < cx.size(); i++) {
        dx.push_back(real_car_x - cx[i]);
        dy.push_back(real_car_y - cy[i]);
        d.push_back(std::hypot(dx[i], dy[i]));
    }
    car_nearest_point_index = static_cast<int>(std::distance(d.begin(), std::min_element(d.begin(), d.end())));
    car_nearest_point_index_obtained = true;

    if (VISUALIZE_ROI_ON_MAP) {
        cv::Mat tmp = this->rois_on_map[this->car_nearest_point_index].clone();
        int car_x = this->m_to_px_x(this->real_car_x);
        int car_y = this->m_to_px_y(this->real_car_y);
        cv::Point center(car_x, car_y);
        int radius = 3;
        cv::circle(     tmp,
                        center,
                        radius,
                        cv::Scalar(0, 0, 255),
                        -1);
        cv::putText(tmp, "real car position", cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
        cv::imshow("ROI on MAP [rrt]", tmp);
        cv::waitKey(20);
    }

    // Publish the current pose
    geometry_msgs::msg::PointStamped odom_point;
    odom_point.header.frame_id = "map";
    odom_point.point.x = this->real_car_x;
    odom_point.point.y = this->real_car_y;
    odom_publisher_->publish(odom_point);
}


void RRTMainNode::raceline_callback(const f1tenth_msgs::msg::RacelineStamped::SharedPtr raceline_msg) {
    this->raceline = raceline_msg->raceline;
    this->raceline_obtained = true;

    auto start_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Processing racing line...");
    this->initialize(raceline);
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(get_logger(), "Raceline processed! [%d ms]", elapsed_time.count());


    // LET'S CALCULATE ALL THE THE ROIs
    start_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Starting Calculating the ROIs...");
    this->rotation_angles                = std::vector<float>        (this->num_of_raceline_points);
    this->rois                           = std::vector<cv::Mat>      (this->num_of_raceline_points);
    this->rois_on_map                    = std::vector<cv::Mat>      (this->num_of_raceline_points);
    this->rois_free_space_polygons       = std::vector<Geometry::Ptr>(this->num_of_raceline_points);
    this->translation_matrices           = std::vector<cv::Mat>      (this->num_of_raceline_points);
    this->inverse_translation_matrices   = std::vector<cv::Mat>      (this->num_of_raceline_points);
    this->rotation_matrices              = std::vector<cv::Mat>      (this->num_of_raceline_points);
    this->inverse_rotation_matrices      = std::vector<cv::Mat>      (this->num_of_raceline_points);
    this->values_to_subtract             = std::vector<cv::Point2f>  (this->num_of_raceline_points);

    int total_size_in_bytes = 0;
    for (int i = 0; i < this->num_of_raceline_points; i++) {
        this->calculate_roi(i);
        total_size_in_bytes += sizeof(this->rois[i]);
    }
    end_time = std::chrono::steady_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(get_logger(), "Finished Calculating the ROIs! [%d ms] [%d bytes]", elapsed_time.count(), total_size_in_bytes);

    // Disable raceline callback after processing it
    raceline_subscriber_.reset();
}


void RRTMainNode::initialize(f1tenth_msgs::msg::Raceline raceline) {
    this->length.assign(raceline.length.begin(), raceline.length.end() - 1);
    this->cx.assign(raceline.x.begin(), raceline.x.end() - 1);
    this->cy.assign(raceline.y.begin(), raceline.y.end() - 1);
    this->heading.assign(raceline.heading.begin(), raceline.heading.end() - 1);
    this->curvature.assign(raceline.curvature.begin(), raceline.curvature.end() - 1);
    this->speed.assign(raceline.speed.begin(), raceline.speed.end() - 1);
    this->width_left.assign(raceline.width_right.begin(), raceline.width_right.end() - 1);
    this->width_right.assign(raceline.width_left.begin(), raceline.width_left.end() - 1);
    this->num_of_raceline_points = this->cx.size();

    // Let's calculate the normals
    double a_x, a_y, b_x, b_y;
    double l_x, l_y, l_norm, r_x, r_y, r_norm;
    for (int i = 0; i < (int)(this->cx.size()); i++) {
        a_x = this->cx[i];
        a_y = this->cy[i];
        if (i != ((int)(this->cx.size()) - 1)) {
            b_x = this->cx[i+1];
            b_y = this->cy[i+1];
        } else {
            b_x = this->cx[0];
            b_y = this->cy[0];
        }
        // We rotate it, a rotation matrix with theta = pi/2 and -pi/2
        r_x = b_y - a_y;
        r_y = a_x - b_x;
        l_x = a_y - b_y;
        l_y = b_x - a_x;
        // We normalize it and multiply by the LATERAL_DISTANCE value
        r_norm = sqrt(r_x*r_x + r_y*r_y);
        r_x = r_x / r_norm * (this->width_right[i] + 0.5 * this->width_right[i]);
        r_y = r_y / r_norm * (this->width_right[i] + 0.5 * this->width_right[i]);
        l_norm = sqrt(l_x*l_x + l_y*l_y);
        l_x = l_x / l_norm * (this->width_left[i] + 0.5 * this->width_left[i]);
        l_y = l_y / l_norm * (this->width_left[i] + 0.5 * this->width_left[i]);
        // We shift back near to A
        r_x = a_x + r_x;
        r_y = a_y + r_y;
        l_x = a_x + l_x;
        l_y = a_y + l_y;
        this->left_x.push_back(l_x);
        this->left_y.push_back(l_y);
        this->right_x.push_back(r_x);
        this->right_y.push_back(r_y);
    }

    // There we put the original map image in a bigger image for not having problem while rotating
    this->map_image = cv::Mat::zeros(
        this->original_map_image.size[0]*2, // HEIGHT
        this->original_map_image.size[1]*2, // WIDTH
        CV_8UC1);

    cv::Rect original_map_image_in_the_bigger_one(
        (int)(this->original_map_image.size[1]/2),  // 0 and 1 are inverted because size is (rows, cols) that respect the (y, x) convention
        (int)(this->original_map_image.size[0]/2),
        this->original_map_image.size[1],
        this->original_map_image.size[0]);

    this->original_map_image.copyTo(this->map_image(original_map_image_in_the_bigger_one));
}


void RRTMainNode::look_ahead_point_index_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    // Called all the time if a path following algo is running
    this->look_ahead_point_index = (int) msg->data;
    this->look_ahead_point_index_obtained = true;

    if (new_trajectory_just_sent) {
        new_trajectory_just_sent = false;
        return;
    }

    if (check_to_send_original_raceline) {
        if (look_ahead_point_index > new_trajectory_end_index) {
            check_to_send_original_raceline = false;

            std_msgs::msg::Header common_header;
            common_header.stamp = this->get_clock()->now();
            common_header.frame_id = "map";

            f1tenth_msgs::msg::RacelineStamped raceline_msg;
            raceline_msg.header = common_header;
            raceline_msg.raceline = this->raceline;

            raceline_publisher_->publish(raceline_msg);
            RCLCPP_INFO(get_logger(), "Original raceline sent back!");
        }
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTMainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
