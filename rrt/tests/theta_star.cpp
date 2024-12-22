#include <rrt/theta_star.hpp>


int main() {
    // Load Map from MAP_FILEPATH
    std::string MAP_FILEPATH = "/home/ubm/repo/maps/8mar/8marzo_edited.pgm";

    // This image reproduces the infinite loop issue
    MAP_FILEPATH = "/home/ubm/repo/rrt_cpp/tests/roi.png";
    ThetaStar aap_calculator;

    // [INFO] [1718127414.342591553] [rrt]: current_car_pose_in_roi.x: 47 - current_car_pose_in_roi.y: 55
    // [INFO] [1718127414.342598876] [rrt]: goal_in_roi.x: 28 - goal_in_roi.y: 8
    // [INFO] [1718127414.342602944] [rrt]: car_nearest_point_index: 36

    cv::Mat map_image = cv::imread(MAP_FILEPATH, cv::IMREAD_GRAYSCALE);
    // cv::imshow("Map", map_image);
    // cv::waitKey(2000000000);

    if (map_image.empty()) {
        printf("Image not found\n");
        return 1;
    }

    cv::Mat pathImg;
    map_image.copyTo(pathImg);
    cv::Mat rgbImage;
    cv::cvtColor(pathImg, rgbImage, cv::COLOR_GRAY2BGR);
    tuple<int, int> goal = tuple<int, int>({47, 55});
    tuple<int, int> start = tuple<int, int>({28, 8});
    // tuple<int, int> start = tuple<int, int>({0, 0});
    tuple<int, int> currVert = {get<1>(goal), get<0>(goal)};
    list<tuple<int, int>> path = aap_calculator.thetaStar(start, goal, map_image);
    // list<tuple<int, int>> path = thetaStar({9, 9}, {0, 0}, map_image);
    for (const auto& coord : path) {
        // pathImg.at<cv::uint8_t>(get<0>(coord), get<1>(coord)) = 112;
        printf("<%d, %d>\n", get<0>(coord), get<1>(coord));
    }

    // cout << "Path size: " << path.size() << endl;

    // tuple<int, int> currParent = aap_calculator.parent[currVert];
    // Build the path
    // while (currVert != tuple<int, int>({get<1>(start), get<0>(start)})) {
    //     // path.push_front(currVert);
    //     cv::line(rgbImage, cv::Point(get<1>(currVert), get<0>(currVert)), cv::Point(get<1>(currParent), get<0>(currParent)), cv::Scalar(255, 0, 0), 1);
    //     currVert = currParent;
    //     currParent = aap_calculator.parent[currVert];
    // }
    // fflush(stdout);
    // cv::imwrite("/home/ubm/repo/rrt_cpp/tests/path.png", rgbImage);
    return 0;
}
