#include <rrt/mean_yaw.hpp>


void rhombus_function(const CoordinateSequence& windowed_geodesic_coords,
                      const Coordinate& geodesic_coord,
                      const AnyAnglePath& path,
                      const int& idx_closest,
                      const int& idx_anterior_mean_yaw_window,
                      const std::vector<float>& segment_lengths,
                      std::vector<float>& yaws,
                      std::vector<float>& weights) {

    int idx_closest_geodesic = idx_closest - idx_anterior_mean_yaw_window;
    LineSegment sgm(windowed_geodesic_coords[idx_closest_geodesic], windowed_geodesic_coords[idx_closest_geodesic + 1]);

    float l = segment_lengths[idx_closest_geodesic];

    float projection_factor = static_cast<float>(sgm.projectionFactor(geodesic_coord));

    float x = projection_factor * l;

    float sgm_weight;
    if (projection_factor < 0.5) {
        // The point is closer to the first half of the segment
        sgm_weight = (1.0 / l) * x + 0.5;

        if (idx_closest == 0) {
            yaws.push_back(path.initial_yaw);
        } else {
            yaws.push_back(path.yaw[idx_closest - 1]);
        }
    } else {
        // The point is closer to the second half of the segment
        sgm_weight = -(1.0 / l) * x + 1.5;

        if (idx_closest == static_cast<int>(path.sgms_size() - 1)) {
            yaws.push_back(path.final_yaw);
        } else {
            yaws.push_back(path.yaw[idx_closest + 1]);
        }
    }

    weights.push_back(1.0 - sgm_weight);
    yaws.push_back(path.yaw[idx_closest]);
    weights.push_back(sgm_weight);
}


void trapezoidal_function(const CoordinateSequence& windowed_geodesic_coords,
                          const Coordinate& geodesic_coord,
                          const AnyAnglePath& path,
                          const int& idx_closest,
                          const int& idx_anterior_mean_yaw_window,
                          const std::vector<float>& segment_lengths,
                          std::vector<float>& yaws,
                          std::vector<float>& weights) {

    int idx_closest_geodesic = idx_closest - idx_anterior_mean_yaw_window;
    LineSegment sgm(windowed_geodesic_coords[idx_closest_geodesic], windowed_geodesic_coords[idx_closest_geodesic + 1]);

    float l = segment_lengths[idx_closest_geodesic];
    float l_other;

    float projection_factor = static_cast<float>(sgm.projectionFactor(geodesic_coord));

    float x = projection_factor * l;
    float x_intersection;

    float sgm_weight;
    float sgm_weight_other;

    const float oblique_fraction = (1.0 - GP::trapezoidal_one_fraction) * 0.5;
    const float oblique_l = oblique_fraction * l;
    float oblique_l_other;

    float a;
    float b;
    float c;
    float d;

    float m;
    float q;

    if (projection_factor < 0.5) {
        // The point is closer to the first half of the segment

        if (idx_closest == 0) {
            a = -oblique_l;
            b = oblique_l;
            c = -oblique_l;
            d = oblique_l;
            yaws.push_back(path.initial_yaw);
        } else {
            l_other = segment_lengths[idx_closest_geodesic - 1];

            find_line_equation(0.0, 0.5, oblique_l, 1.0, m, q);

            x_intersection = find_x_axis_intersection(m, q);
            if (-x_intersection > l_other * 0.5) {
                a = -l_other * 0.5;
                find_line_equation(a, 0.0, 0.0, 0.5, m, q);
                b = find_y_one_intersection(m, q);
            } else {
                a = x_intersection;
                b = oblique_l;
            }

            oblique_l_other = oblique_fraction * l_other;
            find_line_equation(-oblique_l_other, 1.0, 0.0, 0.5, m, q);
            x_intersection = find_x_axis_intersection(m, q);
            if (x_intersection > l * 0.5) {
                d = l * 0.5;
                find_line_equation(d, 0.0, 0.0, 0.5, m, q);
                c = find_y_one_intersection(m, q);
            } else {
                c = -oblique_l_other;
                d = x_intersection;
            }

            yaws.push_back(path.yaw[idx_closest - 1]);
        }

        if (x > d) {
            sgm_weight_other = 0.0;
        } else {
            sgm_weight_other = (d - x) / (d - c);
        }
        weights.push_back(sgm_weight_other);

        yaws.push_back(path.yaw[idx_closest]);
        if (x > b) {
            sgm_weight = 1.0;
        } else {
            sgm_weight = (x - a) / (b - a);
        }
        weights.push_back(sgm_weight);

    } else {
        // The point is closer to the second half of the segment

        if (idx_closest == static_cast<int>(path.sgms_size() - 1)) {
            a = l - oblique_l;
            b = l + oblique_l;
            c = l - oblique_l;
            d = l + oblique_l;
            yaws.push_back(path.final_yaw);
        } else {
            l_other = segment_lengths[idx_closest_geodesic + 1];

            oblique_l_other = oblique_fraction * l_other;
            find_line_equation(l, 0.5, l + oblique_l_other, 1.0, m, q);

            x_intersection = find_x_axis_intersection(m, q);
            if (x_intersection < l * 0.5) {
                a = l * 0.5;
                find_line_equation(a, 0.0, l, 0.5, m, q);
                b = find_y_one_intersection(m, q);
            } else {
                a = x_intersection;
                b = l + oblique_l_other;
            }

            find_line_equation(l - oblique_l, 1.0, l, 0.5, m, q);
            x_intersection = find_x_axis_intersection(m, q);
            if (x_intersection > l + l_other * 0.5) {
                d = l + l_other * 0.5;
                find_line_equation(d, 0.0, l, 0.5, m, q);
                c = find_y_one_intersection(m, q);
            } else {
                c = l - oblique_l;
                d = x_intersection;
            }

            yaws.push_back(path.yaw[idx_closest + 1]);
        }

        if (x < a) {
            sgm_weight_other = 0.0;
        } else {
            sgm_weight_other = (x - a) / (b - a);
        }
        weights.push_back(sgm_weight_other);

        yaws.push_back(path.yaw[idx_closest]);
        if (x < c) {
            sgm_weight = 1.0;
        } else {
            sgm_weight = (d - x) / (d - c);
        }
        weights.push_back(sgm_weight);
    }

}


bool found_mean_yaw(const CoordinateXY& coord,
                    const CoordinateSequence& line_coords,
                    const AnyAnglePath& path,
                    const GeometryFactory& factory,
                    float& mean_yaw,
                    int& idx_closest,
                    geos_t& distance_to_coord) {

    if (line_coords.size() == 2) {
        // If the path has only one segment, return the yaw of that segment
        assert(path.sgms_size() == 1);
        mean_yaw = path.yaw[0];
        idx_closest = 0;
        LineSegment line_segment(line_coords[0], line_coords[1]);
        distance_to_coord = line_segment.distance(coord);
        return true;
    }

    // Create a unique LineSegment and use it to calculate each distance to `coord`.
    std::vector<std::pair<int, geos_t>> distances;  // index, distance
    distances.reserve(path.sgms_size());

    LineSegment line_segment(line_coords[0], line_coords[1]);
    // The `distance` method returns the shortest length between the point and all points of the line segment.
    // Therefore, it's not the perpendicular distance to the line segment.
    distances.emplace_back(0, line_segment.distance(coord));

    for (size_t i = 1; i < line_coords.size() - 1; i++) {
        line_segment.p0 = line_segment.p1;
        line_segment.p1 = line_coords[i + 1];
        distances.emplace_back(i, line_segment.distance(coord));
    }

    // Sort the distances in ascending order
    std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

    // If the first two distances are equal (not more than two and must be adjacent) within a certain tolerance,
    // study the "junction zone" (explained later) of the two segments.

    if (distances.size() > 2 and
        std::abs(distances[0].second - distances[2].second) < GP::eps_L2_eq) {

        // Here at least three distances are equal. We don't handle this case: the path is probably U-shaped and the
        // sample is in the middle of the U. Therefore, the sample is in the inside of the turn. How that sample does
        // not correspond to a ROI's black pixel? Extremely unlikely. We can ignore this case.
        RCLCPP_WARN(rclcpp::get_logger("rrt_cpp"), "The sample is likely in the proximity of a U-shaped path. Ignoring.");
        return false;
    }

    int idx_junction;
    bool is_in_junction_zone = false;
    double junction_zone_offset;
    LineSegment closest_sgm;

    if (std::abs(distances[0].second - distances[1].second) < GP::eps_L2_eq) {
        // For completeness, we add the condition to stop the function if the two closest segments are not adjacent.
        // That would likely mean that the point belongs to the line perpendicular to the segment connecting the first
        // and last points of the path and passing through its middle point.
        // This is a very rare case, because it is most likely a black pixel of the ROI.
        if (std::abs(distances[0].first - distances[1].first) != 1) {
            RCLCPP_WARN(rclcpp::get_logger("rrt_cpp"), "The two closest segments are not adjacent. Ignoring.");
            return false;
        }

        // Here the point belongs to the "junction zone" of two segments. This zone is only defined in the outer
        // side of the turn. It is generated by the angle that the two perpendiculars to the segments at the point of
        // interest (i.e., the junction) form. In the inner side of the turn, the point belongs to the "degenerate
        // junction zone".
        // If the point is in the junction zone, then there are two cases:
        // 1) the point belongs to the line that bisects the junction zone angle, it is possible to immediately return
        //    the mean of the two yaws of the corresponding segments, even if the point is in the degenerate zone;
        // 2) the point does not belong to the line that bisects the junction zone angle, it is necessary to offset the
        //    LineString to make the point lie on it.

        // For case 1: calculate the two oriented perpendicular distances to the segments. If the two distances are
        // equal (within `GP::eps_L2_eq`), the point belongs to the line that bisects the junction zone angle. Return
        // the mean of the two yaws of the corresponding segments.
        LineSegment closest_sgm_anterior(line_coords[distances[0].first], line_coords[distances[0].first + 1]);
        LineSegment closest_sgm_posterior(line_coords[distances[0].first + 1], line_coords[distances[0].first + 2]);
        double perpendicular_distance_anterior = closest_sgm_anterior.distancePerpendicularOriented(coord);
        double perpendicular_distance_posterior = closest_sgm_posterior.distancePerpendicularOriented(coord);

        if (std::abs(perpendicular_distance_anterior - perpendicular_distance_posterior) < GP::eps_L2_eq) {
            mean_yaw = AngleOp::bisect_angle(path.yaw[distances[0].first], path.yaw[distances[1].first]);
            // Could also choose `distances[0]` in the following two lines
            idx_closest = distances[1].first;
            distance_to_coord = distances[1].second;

            RCLCPP_DEBUG(rclcpp::get_logger("rrt_cpp"), "The point is in the junction zone and belongs to the line "
                                                        "that bisects the junction zone angle.");
            return true;
        }

        // For case 2: if the two distances are not equal, the offset is the greater oriented distance in modulus.
        // The offset operation has to be done later.
        is_in_junction_zone = true;
        if (std::abs(perpendicular_distance_anterior) > std::abs(perpendicular_distance_posterior)) {
            idx_closest = distances[0].first;
            idx_junction = idx_closest + 1;
            junction_zone_offset = perpendicular_distance_anterior;
            closest_sgm = closest_sgm_anterior;
        } else {
            idx_closest = distances[1].first;
            idx_junction = idx_closest;
            junction_zone_offset = perpendicular_distance_posterior;
            closest_sgm = closest_sgm_posterior;
        }

    }

    // Otherwise create a LineString with the points of the five segments, which are: the one that is closest to `coord`
    // and the four adjacent ones. If it's not possible to take the adjacent segments because they don't exist, stop to
    // the last possible segment.

    // Take two segments on each side of the closest segment or less if they don't exist.
    // NOTE: here we have at least two segments
    if (not is_in_junction_zone) {
        idx_closest = distances[0].first;
        closest_sgm = LineSegment(line_coords[idx_closest], line_coords[idx_closest + 1]);
    }
    distance_to_coord = distances[idx_closest].second;

    // if (closest_sgm.orientationIndex(coord) == 0) {
    //     RCLCPP_DEBUG(rclcpp::get_logger("rrt_cpp"), "The point is collinear with the closest segment.");
    // }

    // Since the sampled `coord` cannot have projection factor outside the range [0, 1] if the closest segment is the
    // first or last one of the path (this happens if the buffered path line has `CAP_FLAT` buffer end cap style instead
    // of `CAP_ROUND`), the following block is commented out.
    /*
     * if (idx_closest == 0 and closest_sgm.projectionFactor(coord) <= 0.0) {
     *     // The point is in the anterior cap of the line strip, we can return the yaw of the first segment of the path.
     *     mean_yaw = path.yaw[0];
     *     RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "The point is in the cap of the line strip.");
     *     RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "mean_yaw: %f", mean_yaw);
     *     return true;
     * } else if (idx_closest == static_cast<int>(path.sgms_size() - 1) and closest_sgm.projectionFactor(coord) >= 1.0) {
     *     // The point is in the posterior cap of the line strip, we can return the yaw of the last segment of the path.
     *     mean_yaw = path.yaw[path.sgms_size() - 1];
     *     RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "The point is in the cap of the line strip.");
     *     RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "mean_yaw: %f", mean_yaw);
     *     return true;
     * }
     */

    // Here we are not handling possible overflows or underflows of the indices, because it would be unlikely to happen,
    // unless one chooses an absurdly large buffer and mean yaw windows.
    int idx_anterior_buffer_window = idx_closest - GP::buffer_window < 0 ? 0 : idx_closest - GP::buffer_window;
    int idx_anterior_mean_yaw_window = idx_closest - GP::mean_yaw_window < 0 ? 0 : idx_closest - GP::mean_yaw_window;
    int idx_posterior_buffer_window = idx_closest + GP::buffer_window > static_cast<int>(path.sgms_size() - 1) ?
                                      path.sgms_size() - 1 : idx_closest + GP::buffer_window;
    int idx_posterior_mean_yaw_window = idx_closest + GP::mean_yaw_window > static_cast<int>(path.sgms_size() - 1) ?
                                        path.sgms_size() - 1 : idx_closest + GP::mean_yaw_window;

    CoordinateSequence windowed_coords(/*size=*/0, /*dims=*/2);
    for (int i = idx_anterior_buffer_window; i <= idx_posterior_buffer_window + 1; i++) {
        windowed_coords.add(line_coords[i]);
    }

    LineString::Ptr g_line_windowed = factory.createLineString(windowed_coords);
    Point::Ptr p_coord = factory.createPoint(coord);
    CoordinateSequence geodesic_coords;
    Coordinate geodesic_coord;
    bool geodesic_coords_are_offset = false;

    // p_coord->within(g_line_windowed.get()) could return wrong results, see https://github.com/libgeos/geos/issues/968
    if (is_in_junction_zone or not p_coord->isWithinDistance(g_line_windowed.get(), 1e-6)) {
        // The point does not lie on the LineString, so we need to offset the LineString to make the point lie on it.

        // Offset the LineString by the distance to `coord` (with the right sign, in the GEOS library is called orientation).
        // If the resulting LineString has not the same number of points as the original one, offset again the original
        // LineString by a smaller distance. Repeat until the number of points of the offset line is the same or stop after
        // a certain number of iterations.
        double max_buffer_distance;
        if (is_in_junction_zone) {
            max_buffer_distance = junction_zone_offset;
        } else {
            max_buffer_distance = closest_sgm.distancePerpendicularOriented(coord);
        }
        double buffer_distance = max_buffer_distance;
        const double buffer_distance_step = max_buffer_distance * (1.0 / (GP::buffer_retries + 1));

        int retries;
        for (retries = 0; retries <= GP::buffer_retries; retries++) {
            // RCLCPP_INFO(rclcpp::get_logger("rrt_cpp"), "buffer_distance: %.9f", buffer_distance);

            Geometry::Ptr g_line_buffered = OffsetCurve::getCurve(*g_line_windowed,
                                                                  /*buffer_distance=*/buffer_distance,
                                                                  /*quadrantSegments=*/4,  // Not important since no curves need to be generated thanks to JOIN_MITRE
                                                                  BufferParameters::JOIN_MITRE,
                                                                  /*mitreLimit=*/100000.0);  // Very high value to avoid corner beveling

            // Following the Douglas-Peucker algorithm, obtain a new LineString with possibly fewer points.
            // A point within its current candidate simplifying subsegment is removed if its perpendicular distance to
            // that subsegment is less or equal to `GP::simplify_tolerance`. The first and last points of the original
            // line are always kept.
            geodesic_coords = *DouglasPeuckerLineSimplifier::simplify(*(g_line_buffered->getCoordinates()), GP::simplify_tolerance, true);

            if (geodesic_coords.size() == 0) {
                RCLCPP_WARN(rclcpp::get_logger("rrt_cpp"), "The offset LineString has zero points.");
                continue;
            }
            if (geodesic_coords.hasRepeatedOrInvalidPoints()) {
                RCLCPP_WARN(rclcpp::get_logger("rrt_cpp"), "The offset LineString has repeated or invalid points.");
                return false;
            }

            if (geodesic_coords.size() == windowed_coords.size()) {
                geodesic_coords_are_offset = true;

                // Calculate `geodesic_coord`
                if (retries == 0) {
                    geodesic_coord = coord;
                } else {
                    if (not is_in_junction_zone) {
                        double coord_pos = closest_sgm.projectionFactor(coord);
                        // Will throw IllegalStateException	if the segment has zero length (should never happen)
                        closest_sgm.pointAlongOffset(coord_pos, buffer_distance, geodesic_coord);
                    } else {
                        LineSegment hypotenuse(static_cast<Coordinate>(coord), line_coords[idx_junction]);
                        // Will throw IllegalStateException	if the segment has zero length (should never happen)
                        LineSegment parallel = closest_sgm.offset(buffer_distance);
                        geodesic_coord = hypotenuse.lineIntersection(parallel);
                    }
                }
                break;
            } else {
                buffer_distance -= buffer_distance_step;
            }
        }

        if (not geodesic_coords_are_offset) {
            geodesic_coords = windowed_coords;
            if (not is_in_junction_zone) {
                closest_sgm.project(static_cast<Coordinate>(coord), geodesic_coord);
            } else {
                // Here the projection of `coord` to the windowed path is the junction point
                mean_yaw = AngleOp::bisect_angle(path.yaw[idx_junction - 1], path.yaw[idx_junction]);
                return true;
            }
        }

    } else {
        // The point lies on the LineString, so we can use the LineString as it is. Skip the offset operation.
        geodesic_coords = windowed_coords;
        geodesic_coord = coord;
    }

    // Now use the trapezoidal (or rhombus) membership functions to calculate the weight to assign to each yaw.
    // (NOTE: here the functions are considering only the closest and second-closest segments instead of the whole line,
    // because we are assuming that the other segments will not have an impact on the yaw estimation. We are relaxing
    // the problem to make it easier to solve).

    // Select only the geodesic coordinates that are within the `GP::mean_yaw_window` of the closest segment.
    // Convert `idx_anterior_mean_yaw_window` and `idx_posterior_mean_yaw_window` to the indices of `geodesic_coords`.
    // GEOS does not easily allow to remove elements from a CoordinateSequence, so we will create a new one.
    CoordinateSequence windowed_geodesic_coords(/*size=*/0, /*dims=*/2);
    for (int i = idx_anterior_mean_yaw_window - idx_anterior_buffer_window;
            i <= idx_posterior_mean_yaw_window - idx_anterior_buffer_window + 1; i++) {

        windowed_geodesic_coords.add(geodesic_coords[i]);
    }

    // Debug only
    // if (not factory.createPoint(geodesic_coord)->within(factory.createLineString(windowed_geodesic_coords)->buffer(1e-6).get())) {
    //     RCLCPP_WARN(rclcpp::get_logger("rrt_cpp"), "The geodesic point is not within the buffered windowed geodesic coords.");
    //     return false;
    // }

    std::vector<float> segment_lengths;
    if (geodesic_coords_are_offset) {
        segment_lengths.reserve(windowed_geodesic_coords.size() - 1);
        for (size_t i = 0; i < windowed_geodesic_coords.size() - 1; i++) {
            geos_t dx = windowed_geodesic_coords[i + 1].x - windowed_geodesic_coords[i].x;
            geos_t dy = windowed_geodesic_coords[i + 1].y - windowed_geodesic_coords[i].y;
            segment_lengths.push_back(static_cast<float>(std::sqrt(dx * dx + dy * dy)));
        }
    } else {
        segment_lengths.assign(path.segment_length.begin() + idx_anterior_mean_yaw_window,
                               path.segment_length.begin() + idx_posterior_mean_yaw_window + 1);
    }

    std::vector<float> yaws;
    std::vector<float> weights;

    if (GP::use_rhombus_function) {
        rhombus_function(windowed_geodesic_coords, geodesic_coord, path,
                         idx_closest, idx_anterior_mean_yaw_window,
                         segment_lengths, yaws, weights);
    } else {
        trapezoidal_function(windowed_geodesic_coords, geodesic_coord, path,
                             idx_closest, idx_anterior_mean_yaw_window,
                             segment_lengths, yaws, weights);
    }

    // Calculate the mean of the three yaws weighted by the membership functions (normalize the weights to sum to 1).
    mean_yaw = AngleOp::weighted_circular_mean(yaws, weights);
    return true;
}
