/**
 * @file theta_star.hpp
 * @brief This header file contains the classes and functions declarations required for the Theta* algorithm implementation.
 */

#pragma once

#include <opencv2/opencv.hpp>

#define NULL_PARENT tuple<int, int>({-1, -1})
#define OCCUPIED 0

using namespace std;


namespace std {
    template <>
    struct hash<std::tuple<int, int>> {
        std::size_t operator()(const std::tuple<int, int>& k) const {
            return std::hash<int>()(std::get<0>(k)) ^ std::hash<int>()(std::get<1>(k) << 1);
        }
    };
}


/**
 * @brief Implemetation of the Theta* algorithm.
 * 
 * @note For `vertex` at coords `x` and `y` we mean the top-left vertex of the pixel at coords `x` and `y`.
 */
class ThetaStar {
    public:
    ThetaStar() {}

    /**
     * @brief Comparator class used to hold the vertices of the `open` set in ascending order
     *        based on their estimated distance from the goal.
     */
    class Comparator {
    public:
        Comparator() {}
        Comparator(ThetaStar* ts_input) : ts(ts_input) {}

        // Used to sort the priority queue in ascending order based on the path cost
        // Note: depends on `gCost` and `hCost`
        bool operator() (tuple<int, int> l, tuple<int, int> r) {
            return ts->gCost.at<float>(get<0>(l), get<1>(l)) + hCost(l, ts->goalVert) < ts->gCost.at<float>(get<0>(r), get<1>(r)) + hCost(r, ts->goalVert);
        }

    private:
        ThetaStar* ts;
    };

    cv::Mat gCost;                                           // matrix containing the cost of the path to reach each vertex
    tuple<int, int> goalVert;                                // goal vertex global variable, used in the comparison of vertex costs
    unordered_map<tuple<int, int>, tuple<int, int>> parent;  // hash map that contains the parent of each vertex
    set<tuple<int, int>, Comparator> open;                   // ordered set of reachable vertices, in ascending order of distance from the goal
    list<tuple<int, int>> closed;                            // list of vertexes that have already been explored
    cv::Mat mapImage;

    /**
     * @brief Calculate straight-line distance between the vertex `s` and `goal` vertex,
     *        used as an estimation of the cost of the path between `s` and `goal`.
     * 
     * @param s         the vertex
     * @param goal      the goal vertex
     * @return float 
     */
    static float hCost(const tuple<int, int> s, const tuple<int, int> goal) {
        return sqrt((pow((get<0>(s) - get<0>(goal)) ,2)) + (pow((get<1>(s) - get<1>(goal)), 2)));
    }

    /**
     * @brief Check for line of sight between two vertices.
     * 
     * @param a         The first vertex
     * @param b         The second vertex
     * @param map       The occupancy map.
     * @return true     If the vertex a has line of sight with vertex b.
     * @return false    If the vertex a does not have line of sight with vertex b.
     */
    static bool lineOfSight(tuple<int, int> a, tuple<int, int> b, const cv::Mat& map) {
        float x0 = get<0>(a);
        float y0 = get<1>(a);
        float x1 = get<0>(b);
        float y1 = get<1>(b);

        float dy = y1 - y0;
        float dx = x1 - x0;

        float sy;
        float sx;
        float f = 0;

        if (dy < 0) {
            // a is to the right of b
            dy = -dy;
            sy = -1;
        } else {
            sy = 1;
        }

        if (dx < 0) {
            // a is below b
            dx = -dx;
            sx = -1;
        } else {
            sx = 1;
        }

        if (dx >= dy) {
            while (x0 != x1) {
                f = f + dy;
                if (f >= dx) {
                    if (map.at<cv::uint8_t>(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)) == OCCUPIED) return false;
                    y0 = y0 + sy;
                    f = f - dx;
                }

                if ((f != 0) && (map.at<cv::uint8_t>(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)) == OCCUPIED)) return false;
                if ((dy == 0) && (map.at<cv::uint8_t>(x0 + ((sx - 1) / 2), y0) == OCCUPIED) && (map.at<cv::uint8_t>(x0 + ((sx - 1) / 2), y0 - 1) == OCCUPIED)) return false;
                x0 = x0 + sx;
            }
        } else {
            while (y0 != y1) {
                f = f + dx;
                if (f >= dy) {
                    if (map.at<cv::uint8_t>(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)) == OCCUPIED) return false;
                    x0  = x0 + sx;
                    f = f - dy;
                }
                if ((f != 0) && (map.at<cv::uint8_t>(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)) == OCCUPIED)) return false;
                if ((dx == 0) && (map.at<cv::uint8_t>(x0, y0 + ((sy - 1) / 2)) == OCCUPIED) && (map.at<cv::uint8_t>(x0 - 1, y0 + ((sy - 1) / 2)) == OCCUPIED)) return false;

                y0 = y0 + sy;
            }
        }
        return true;
    }

    /**
     * @brief Get all neighbors of a vertex (we consider an 8-connected space, so diagonal neighbors included).
     * 
     * @param s         Vertex
     * @param mapSize   The size of the map.
     * @return list<tuple<int, int>> List of neighbors
     */
    static list<tuple<int, int>> neighbors(const tuple<int, int> s, const tuple<int, int>& mapSize) {
        list<tuple<int, int>> neighborList = list<tuple<int, int>>();
        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                if ((i == 0 && j == 0)   // the node itself is not a neighbor
                    || get<0>(s) + i < 0  || get<1>(s) + j < 0  // the coordinate cannot be negative
                    // the coordinate cannot be bigger than the map size
                    || get<0>(s) + i >= get<0>(mapSize)
                    || get<1>(s) + j >= get<1>(mapSize)) {
                    continue;
                }
                neighborList.emplace_front(get<0>(s) + i, get<1>(s) + j);
            }
        }
        return neighborList;
    }

    /**
     * @brief Calculate the lenght of the straight line between two vertices, in pixels.
     * 
     * @param a     The first vertex.
     * @param b     The second vertex.
     * @return float The distance between the two vertices as a straigth line in pixels.
     */
    static float straightLinecost(const tuple<int, int> a, const tuple<int, int> b) {
        return sqrt(pow(get<0>(a) - get<0>(b), 2) + pow(get<1>(a) - get<1>(b), 2));
    }

    /**
     * @brief Update routine of the theta* algorigthm, updates the `parent` of the current neighbor to either
     *        the `parent` of the `s` vertex if it has line of sight or to the `s` vertex.
     * @note  Depends on `gCost` and `mapImage`
     * 
     * @param s         The vertex that is being currently expanded.
     * @param sNeighbor The neighbor of `s`.
     * @param map       The occupancy map.
     */
    void updateVertex(const tuple<int, int> s, const tuple<int, int> sNeighbor, const cv::Mat& map) {
        tuple<int, int> sParent = parent[s];
        if (lineOfSight(sParent, sNeighbor, map)) {
            if (gCost.at<float>(get<0>(sParent), get<1>(sParent)) + straightLinecost(sParent, sNeighbor) < gCost.at<float>(get<0>(sNeighbor), get<1>(sNeighbor))) {
                parent[sNeighbor] = sParent;
                // If sNeighbor is in open queue remove it
                open.erase(sNeighbor);
                gCost.at<float>(get<0>(sNeighbor), get<1>(sNeighbor)) = gCost.at<float>(get<0>(sParent), get<1>(sParent)) + straightLinecost(sParent, sNeighbor);
                open.insert(sNeighbor);
            }
        }
        else if (lineOfSight(s, sNeighbor, map)) {
            if (gCost.at<float>(get<0>(s), get<1>(s)) + straightLinecost(s, sNeighbor) < gCost.at<float>(get<0>(sNeighbor), get<1>(sNeighbor))) {
                parent[sNeighbor] = s;
                open.erase(sNeighbor);
                gCost.at<float>(get<0>(sNeighbor), get<1>(sNeighbor)) = gCost.at<float>(get<0>(s), get<1>(s)) + straightLinecost(s, sNeighbor);
                open.insert(sNeighbor);
            }
        }
    }

    /**
     * @brief Main function of the theta* algorithm.
     * 
     * @note  Within the function we reverse the `x` and `y` coordinates of the vertices for compatability
     *        with the `lineOfSight` function. This detail should be ignored outside of the `thetaStar` function.
     * 
     * @param start     The start vertex.
     * @param goal      The goal vertex.
     * @param map       The occupancy map, binary image.
     * @return list<tuple<int, int>> The path from the start vertex to the goal vertex, empty if not found.
     */
    list<tuple<int, int>> thetaStar(const tuple<int, int> start, tuple<int, int> goal, const cv::Mat& map) {
        parent = unordered_map<tuple<int, int>, tuple<int, int>>();
        parent.clear();

        gCost = cv::Mat::zeros(map.rows, map.cols, CV_32F);
        tuple<int, int> mapSize = {map.rows, map.cols};
        mapImage = map;
        tuple<int, int> reversedStart = {get<1>(start), get<0>(start)};
        goalVert = {get<1>(goal), get<0>(goal)};  // Needed for hCost calculation in comparator class
        parent[goalVert] = NULL_PARENT;
        parent[reversedStart] = reversedStart;

        closed = list<tuple<int, int>>();
        Comparator comp(this);
        open = set<tuple<int, int>, Comparator>(comp);

        open.insert(reversedStart);

        while (!open.empty()) {
            tuple<int, int> s = *(open.begin());
            open.erase(s);

            if (s == goalVert) {
                break;
            }

            closed.push_front(s);

            list<tuple<int, int>> neighborList = neighbors(s, mapSize);

            for (const auto& neighbor : neighborList) {
                if (find(closed.begin(), closed.end(), neighbor) == closed.end()) {
                    if (open.find(neighbor) == open.end()) {
                        gCost.at<float>(get<0>(neighbor), get<1>(neighbor)) = INFINITY;
                        parent[neighbor] = NULL_PARENT;
                        updateVertex(s, neighbor, map);
                    }
                }
            }
        }

        list<tuple<int, int>> path;
        tuple<int, int> currVert = goalVert;
        tuple<int, int> currParent = parent[goalVert];
        if (currParent == NULL_PARENT) {
            cout << "Goal not found" << endl;
            fflush(stdout);
            // The goal vertex was not reached
              closed.clear();
              parent.clear();
              open.clear();
            return path;
        }
        // Build the path
        while (currVert != reversedStart) {
            // TODO: following line is for debugging purposes. To be removed
            // cout << "Current vertex: " << get<0>(currVert) << ", " << get<1>(currVert) << endl;
            path.emplace_front(get<1>(currVert), get<0>(currVert));
            currVert = currParent;
            currParent = parent[currVert];
        }
        path.push_front(start);
        closed.clear();
        parent.clear();
        open.clear();
        return path;
    }
};
