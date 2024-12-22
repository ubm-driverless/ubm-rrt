#include <rrt/nn_search.hpp>


bool is_already_inserted_xy(const CoordinateSequence& samples,
                            const CoordinateXY& coord,
                            std::vector<CoordinateXY>& X_near,
                            CoordinateXY& closest_point,
                            const bool& fill_fn_params,
                            const geos_t& eps_L2_squared,
                            const geos_t& delta_R_squared) {

    if (samples.getDimension() != 2) {
        throw std::runtime_error("The dimension of the `CoordinateSequence` obj is not 2.");
    }

    geos_t minimum_distance = std::numeric_limits<geos_t>::max();

    for (size_t i = 0; i < samples.size(); i++) {
        CoordinateXY this_coord = samples[i];
        geos_t L2_squared = (this_coord.x - coord.x) * (this_coord.x - coord.x) +
                            (this_coord.y - coord.y) * (this_coord.y - coord.y);

        if (L2_squared < eps_L2_squared) {
            return true;
        }

        if (fill_fn_params) {
            if (L2_squared < minimum_distance) {
                minimum_distance = L2_squared;
                closest_point = this_coord;
            }
            if (L2_squared < delta_R_squared) {
                X_near.push_back(this_coord);
            }
        }
    }

    return false;
}


bool is_already_inserted_xy_flann_tree(flann::Index<flann::L2_Simple<geos_t>>& kd_tree,
                                       const flann::Matrix<geos_t>& query,
                                       flann::Matrix<int>& k_indices,
                                       flann::Matrix<geos_t>& k_distances,
                                       const std::vector<RRTNode>& rrt_tree,
                                       std::vector<std::reference_wrapper<const RRTNode>>& X_near,
                                       std::reference_wrapper<const RRTNode>& closest_point,
                                       const bool& fill_fn_params,
                                       const geos_t& eps_L2_squared,
                                       const geos_t& delta_R_squared,
                                       const int& max_neighbors) {

    // `flann::SearchParams(-1)` means exact search
    // `k_distances` contains squared Euclidean distances
    int n_found = kd_tree.knnSearch(query, k_indices, k_distances, /*knn=*/max_neighbors,
                                    flann::SearchParams(-1));
    if (n_found == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rrt_cpp"), "No points found in the flann tree. Impossible to happen.");
        throw std::runtime_error("No points found in the flann tree. Impossible to happen.");
    }

    geos_t first_distance = k_distances[0][0];
    if (first_distance < eps_L2_squared) {
        return true;
    }

    int index;
    if (fill_fn_params) {
        index = k_indices[0][0];
        // `closest_point = std::cref(rrt_tree[index]);` or
        closest_point = rrt_tree[index];
        if (first_distance < delta_R_squared) {
            X_near.push_back(closest_point);
        } else {
            return false;
        }
    } else {
        return false;
    }

    for (int i = 1; i < n_found; i++) {
        // If the index is -1, it means that there are no more points found within radius `sqrt(delta_R_squared)`
        index = k_indices[0][i];

        if (k_distances[0][i] < delta_R_squared) {
            X_near.push_back(rrt_tree[index]);
        } else {
            break;
        }
    }

    return false;
}
