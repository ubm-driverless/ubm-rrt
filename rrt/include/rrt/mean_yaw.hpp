/**
 * @file mean_yaw.hpp
 * @brief This header file contains the functions declarations related to computing the mean yaw of a just sampled point
 * considering the any-angle path.
 */

#pragma once

#include <geos/operation/buffer/OffsetCurve.h>
#include <geos/simplify/DouglasPeuckerLineSimplifier.h>

#include <rrt/rrt.hpp>

using geos::operation::buffer::OffsetCurve;
using geos::simplify::DouglasPeuckerLineSimplifier;


void rhombus_function(const CoordinateSequence& windowed_geodesic_coords,
                      const Coordinate& geodesic_coord,
                      const AnyAnglePath& path,
                      const int& idx_closest,
                      const int& idx_anterior_mean_yaw_window,
                      const std::vector<float>& segment_lengths,
                      std::vector<float>& yaws,
                      std::vector<float>& weights);

inline void find_line_equation(float x1, float y1, float x2, float y2, float& m, float& q) {
    // We are assuming that the line is neither vertical nor horizontal
    m = (y2 - y1) / (x2 - x1);
    q = y1 - m * x1;
}

inline float find_x_axis_intersection(float m, float q) {
    // We are assuming that the line is neither vertical nor horizontal
    return -q / m;
}

inline float find_y_one_intersection(float m, float q) {
    // We are assuming that the line is neither vertical nor horizontal
    return (1.0 - q) / m;
}

void trapezoidal_function(const CoordinateSequence& windowed_geodesic_coords,
                          const Coordinate& geodesic_coord,
                          const AnyAnglePath& path,
                          const int& idx_closest,
                          const int& idx_anterior_mean_yaw_window,
                          const std::vector<float>& segment_lengths,
                          std::vector<float>& yaws,
                          std::vector<float>& weights);

/**
 * When trying to understand this function, see images `junction_zone.svg` and `projection_in_junction_zone.svg` in the
 * `rrt_cpp` package as a reference. Good luck!
 * TODO: add a more detailed explanation
 * @param coord
 * @param line_coords
 * @param path
 * @param factory
 * @param mean_yaw
 * @return
 */
bool found_mean_yaw(const CoordinateXY& coord,
                    const CoordinateSequence& line_coords,
                    const AnyAnglePath& path,
                    const GeometryFactory& factory,
                    float& mean_yaw,
                    int& idx_closest,
                    geos_t& distance_to_coord);
