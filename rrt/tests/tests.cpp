#include <rrt/rrt.hpp>
#include <rrt/nn_search.hpp>
#include <rrt/mean_yaw.hpp>
#include <rrt/steer.hpp>


float GP::strip_width = 1.0;
int GP::n_iterations = 1000;
geos_t GP::eps_L2_squared = 0.005;
geos_t GP::delta_R_squared = 2.0;
int GP::max_neighbors = 10;
geos_t GP::eps_L2_eq = 1e-30;
double GP::simplify_tolerance = 0.01;
int GP::buffer_window = 2;
int GP::mean_yaw_window = 1;
int GP::buffer_retries = 3;
bool GP::use_rhombus_function = true;
float GP::trapezoidal_one_fraction = 0.8;
bool GP::use_orientation_bias = true;
float GP::orientation_bias = 0.1;
bool GP::enable_X_free_sampling = true;
int GP::X_free_interval_over_nodes = 15;
int GP::X_free_interval_over_iterations = 30;
float GP::goal_region_radius = 0.3;
float GP::w_e = 0.2;
float GP::w_theta = 0.8;
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
float GP::yaw_tolerance = 0.53;
int GP::downsample_factor = 3;

float GP::drag_coeff = 0.2;
float GP::m_veh = 3.158;
float GP::v_max = 11.0;
float GP::dyn_model_exp = 1.0;


void radius_nn_search_linear(const Geometry& g_strip, const GeometryFactory& factory);
void radius_nn_search_kd_tree_flann(const Geometry& g_strip, const AnyAnglePath& path, const GeometryFactory& factory);
void mean_yaw_calculation(const CoordinateSequence& line_coords, const AnyAnglePath& path, const Geometry& g_strip, const GeometryFactory& factory);


std::vector<float> linspace(const float start, const float end, const int num) {
    std::vector<float> linspaced(num);
    float delta = (end - start) / (float(num) - 1);

    for(int i = 0; i < num; i++) {
        linspaced[i] = start + delta * i;
    }

    return linspaced;
}


void create_csv_mean_yaw(std::vector<std::tuple<float, float, float>> data) {
    std::stringstream ss;
    std::ofstream file("/home/ubm/repo/rrt_cpp/tests/mean_yaw.csv");

    for (size_t i = 0; i < data.size(); i++) {
        ss << std::fixed << std::setprecision(5) <<
        std::get<0>(data[i]) << "," << std::get<1>(data[i]) << "," << std::get<2>(data[i]) << "\n";
        file << ss.str();
        ss.str("");
    }
    file.close();
}


int main() {

    GP::assert_params();

    std::list<std::tuple<int, int>> path = {
            {0, 0}, {10, 10}, {30, 2}, {32, 50},
            {36, 50}, {40, 20}, {44, 50}, {50, 50}
    };

    // std::vector<std::pair<geos_t, geos_t>> path = {
    //         {-2.0, 0.5}, {1.0, 1.0}, {3.0, 0.2}, {3.2, 5.0},
    //         {3.6, 5.0}, {4.0, 2.0}, {4.4, 5.0}, {5.0, -8.0}
    // };
    AnyAnglePath::Ptr aapm = AnyAnglePath::process_theta_star_output(path, M_PI_2, M_PI_4);

    const GeometryFactory::Ptr factory = GeometryFactory::create();

    // dims = 2 because we are working with 2D points
    CoordinateSequence points(/*size=*/0, /*dims=*/2);

    for (size_t i = 0; i < aapm->pts_size(); i ++) {
        points.add(aapm->x[i], aapm->y[i]);
    }

    LineString::Ptr g_line = factory->createLineString(points);
    Geometry::Ptr g_strip = g_line->buffer(GP::strip_width / 4, /*quadrantSegments=*/4, BufferParameters::CAP_FLAT);

    // Get the rectangle that bounds the strip
    const Envelope* envelope;
    envelope = g_strip->getEnvelopeInternal();
    if (envelope->isNull()) {
        throw std::runtime_error("Envelope is null. Something went wrong.");
    }
    if (not envelope->isFinite()) {
        throw std::runtime_error("Envelope is not finite. Something went wrong.");
    }

    // Radius Nearest Neighbors search tests
    radius_nn_search_linear(*g_strip, *factory);
    radius_nn_search_kd_tree_flann(*g_strip, *aapm, *factory);

    // Mean yaw calculation tests
    mean_yaw_calculation(points, *aapm, *g_strip, *factory);
}


void radius_nn_search_linear(const Geometry& g_strip, const GeometryFactory& factory) {
    std::cout << "Radius Nearest Neighbors search linear test" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    int seed = 42;
    std::mt19937 gen(seed);

    std::uniform_real_distribution<> dis_x(-10.0, 10.0);
    std::uniform_real_distribution<> dis_y(-10.0, 10.0);

    CoordinateSequence samples(/*size=*/0, /*dims=*/2);
    samples.add(0.0, 0.0);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < GP::n_iterations; ) {
        CoordinateXY coord_rand(dis_x(gen), dis_y(gen));
        Point::Ptr point_rand = factory.createPoint(coord_rand);

        if (not point_rand->within(&g_strip)) {
            continue;
        }

        // Find X_near set according to Euclidean distance using nearest neighbor search,
        // but if point_rand is too close to any point in the tree, skip this iteration,
        // because it will have a similar yaw to the already existing point
        std::vector<CoordinateXY> X_near;
        CoordinateXY closest_point;
        if (is_already_inserted_xy(samples, coord_rand, X_near, closest_point)) {
            continue;
        }

        // RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "Closest point: %f, %f", closest_point.x, closest_point.y);
        // for (size_t j = 0; j < X_near.size(); j++) {
        //     RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "X_near[%d]: %f, %f", j, X_near[j].x, X_near[j].y);
        // }

        // Add the new point to the samples
        samples.add(coord_rand.x, coord_rand.y);
        i++;
        // std::cout << "Point " << i << ": " << coord_rand.x << ", " << coord_rand.y << std::endl;

        if (i == 57 or i == 657 or i == 888) {
            std::cout << "Point " << i << ": " << coord_rand.x << ", " << coord_rand.y << std::endl;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << std::endl << "Elapsed time: " << elapsed.count() << " s\n";
    std::cout << "--------------------------------------------" << std::endl << std::endl;
}


void radius_nn_search_kd_tree_flann(const Geometry& g_strip, const AnyAnglePath& path, const GeometryFactory& factory) {
    std::cout << "Radius Nearest Neighbors search KD-tree test with FLANN library" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    int seed = 42;
    std::mt19937 gen(seed);

    std::uniform_real_distribution<> dis_x(-10.0, 10.0);
    std::uniform_real_distribution<> dis_y(-10.0, 10.0);

    // Create the KD-tree
    flann::Matrix<geos_t> dataset(new geos_t[2], 1, 2);
    dataset[0][0] = 0.0;
    dataset[0][1] = 0.0;
    flann::Matrix<int> k_indices(new int[GP::max_neighbors], 1, GP::max_neighbors);
	flann::Matrix<geos_t> k_distances(new geos_t[GP::max_neighbors], 1, GP::max_neighbors);
    flann::Matrix<geos_t> query(new geos_t[2], 1, 2);

    flann::Index<flann::L2_Simple<geos_t>> kd_tree(dataset, flann::KDTreeIndexParams(1));
    kd_tree.buildIndex();

    // Create the RRT tree
    std::vector<RRTNode> rrt_tree;
    rrt_tree.emplace_back(path.x[0], path.y[0], path.yaw[0], 0, 0, 0.0, 0.0);
    rrt_tree[0].set_to_root();

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < GP::n_iterations; ) {
        CoordinateXY coord_rand(dis_x(gen), dis_y(gen));
        Point::Ptr point_rand = factory.createPoint(coord_rand);

        if (not point_rand->within(&g_strip)) {
            continue;
        }

        // Find X_near set according to Euclidean distance using nearest neighbor search,
        // but if point_rand is too close to any point in the tree, skip this iteration,
        // because it will have a similar yaw to the already existing point
        std::vector<std::reference_wrapper<const RRTNode>> X_near;
        std::reference_wrapper<const RRTNode> closest_point(RRTNode::get_dummy());

        query[0][0] = coord_rand.x;
        query[0][1] = coord_rand.y;
        if (is_already_inserted_xy_flann_tree(kd_tree, query, k_indices, k_distances, rrt_tree,
                                              X_near, closest_point)) {
            continue;
        }

        // RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "Closest point: %f, %f", closest_point.x, closest_point.y);
        // for (size_t j = 0; j < X_near.size(); j++) {
        //     RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "X_near[%d]: %f, %f", j, X_near[j].x, X_near[j].y);
        // }

        // Add the new point to the samples
        add_point_to_flann_tree(kd_tree, query);
        i++;
        rrt_tree.emplace_back(coord_rand.x, coord_rand.y, /*yaw=*/0.0, /*index=*/i, 0, 0.0, 0.0);
        // std::cout << "Point " << i << ": " << query[0][0] << ", " << query[0][1] << std::endl;

        if (i == 57 or i == 657 or i == 888) {
            std::cout << "Point " << i << ": " << query[0][0] << ", " << query[0][1] << std::endl;
            std::cout << "Nearest point " << closest_point.get().get_index() << ": " << closest_point.get().get_x() << ", " << closest_point.get().get_y() << std::endl;
        }
    }

    delete[] dataset.ptr();
    delete[] query.ptr();
    delete[] k_indices.ptr();
    delete[] k_distances.ptr();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << std::endl << "Elapsed time: " << elapsed.count() << " s\n";
    std::cout << "--------------------------------------------" << std::endl << std::endl;
}


void mean_yaw_calculation(const CoordinateSequence& line_coords,
                          const AnyAnglePath& path,
                          const Geometry& g_strip,
                          const GeometryFactory& factory) {

    std::cout << "         Mean yaw calculation test         " << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    // Create a grid of samples
    std::vector<float> xs = linspace(-0.1, 5.1, 100);
    std::vector<float> ys = linspace(-0.1, 5.1, 100);
    std::vector<CoordinateXY> samples_grid;
    for (size_t i = 0; i < xs.size(); i++) {
        for (size_t j = 0; j < ys.size(); j++) {
            samples_grid.push_back(CoordinateXY(xs[i], ys[j]));
        }
    }

    std::vector<std::tuple<float, float, float>> samples_with_yaw;  // x, y, yaw

    auto start = std::chrono::high_resolution_clock::now();

    float mean_yaw;
    for (size_t i = 0; i < samples_grid.size(); i++) {
        Point::Ptr point = factory.createPoint(samples_grid[i]);

        if (not point->within(&g_strip)) {
            continue;
        }

        int idx_closest;
        geos_t distance_to_coord;
        if (not found_mean_yaw(samples_grid[i], line_coords, path, factory, mean_yaw,
                               idx_closest, distance_to_coord)) {
            continue;
        }

        samples_with_yaw.emplace_back(samples_grid[i].x, samples_grid[i].y, mean_yaw);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << std::endl << "Elapsed time: " << elapsed.count() << " s\n";
    std::cout << "--------------------------------------------" << std::endl << std::endl;

    create_csv_mean_yaw(samples_with_yaw);
}
