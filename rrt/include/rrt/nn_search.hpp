/**
 * @file nn_search.hpp
 * @brief This header file contains the functions declarations related to nearest neighbor search to find the closest
 * nodes in the RRT tree wrt a random sample.
 */

#pragma once

#include <flann/flann.hpp>

#include <rrt/rrt.hpp>


/**
 * Checks if a point is already inserted in the `samples` vector with a certain tolerance `eps_L2_squared`.
 * Fills both the `X_near` vector with the points that within a ball of radius `sqrt(delta_R_squared)` centered at
 * `coord` and the `closest_point` in `samples` wrt to `coord` if `fill_fn_params` is set to `true`.
 *
 * @param samples
 * @param coord The point to check if it is already inserted in `samples`.
 * @param X_near When the function returns `false`, it contains the points in `samples` within `delta_R` centered at
 *               `coord` if `fill_fn_params` is set to `true`; otherwise, it is undefined.
 * @param closest_point When the function returns `false`, it contains the closest point in `samples` to `coord` if
 *                      `fill_fn_params` is set to `true`; otherwise, it is undefined.
 * @param fill_fn_params
 * @param eps_L2_squared Squared Euclidean distance tolerance.
 * @param delta_R_squared Squared radius of the ball centered at `coord`.
 * @return `true` if `coord` is too close to any point in `samples` with a tolerance of `eps_L2_squared`; otherwise,
 *         `false`.
 */
bool is_already_inserted_xy(const CoordinateSequence& samples,
                            const CoordinateXY& coord,
                            std::vector<CoordinateXY>& X_near,
                            CoordinateXY& closest_point,
                            const bool& fill_fn_params = true,
                            const geos_t& eps_L2_squared = GP::eps_L2_squared,
                            const geos_t& delta_R_squared = GP::delta_R_squared);

bool is_already_inserted_xy_flann_tree(flann::Index<flann::L2_Simple<geos_t>>& kd_tree,
                                       const flann::Matrix<geos_t>& query,
                                       flann::Matrix<int>& k_indices,
                                       flann::Matrix<geos_t>& k_distances,
                                       const std::vector<RRTNode>& rrt_tree,
                                       std::vector<std::reference_wrapper<const RRTNode>>& X_near,
                                       std::reference_wrapper<const RRTNode>& closest_point,
                                       const bool& fill_fn_params = true,
                                       const geos_t& eps_L2_squared = GP::eps_L2_squared,
                                       const geos_t& delta_R_squared = GP::delta_R_squared,
                                       const int& max_neighbors = GP::max_neighbors);

inline void add_point_to_flann_tree(flann::Index<flann::L2_Simple<geos_t>>& kd_tree,
                                    const flann::Matrix<geos_t>& query) {
    flann::Matrix<geos_t> query_copy(new geos_t[2], 1, 2);
    query_copy[0][0] = query[0][0];
    query_copy[0][1] = query[0][1];
    kd_tree.addPoints(query_copy);
}
