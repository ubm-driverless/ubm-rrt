cmake_minimum_required(VERSION 3.5)
project(rrt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_definitions(-DUSE_UNSTABLE_GEOS_CPP_API)

# Include the headers
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f1tenth_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)
find_package(geos REQUIRED)
find_package(nanoflann REQUIRED)
find_package(flann REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(lz4 REQUIRED IMPORTED_TARGET liblz4)


if(LZ4_FOUND)
    include_directories(${LZ4_INCLUDE_DIRS})
endif()

if(BLAS_FOUND)
        message(STATUS "Found OpenBLAS library: ${BLAS_LIBRARIES}")
        include_directories(${BLAS_INCLUDE_DIRS})
        link_libraries(${BLAS_LIBRARIES})
else()
        message(FATAL_ERROR "BLAS not found")
endif()

add_executable(rrt_node
               src/rrt_node.cpp
               src/rrt.cpp
               src/roi.cpp
               src/obstacle.cpp
               src/nn_search.cpp
               src/mean_yaw.cpp
               src/steer.cpp
               src/speed_profile.cpp
)
add_executable(theta_star
               tests/theta_star.cpp
)
add_executable(speed_profile
               src/speed_profile.cpp
               tests/speed_profile.cpp
)
add_executable(curvature
               tests/curvature.cpp
)
add_executable(tests
               tests/tests.cpp
               src/nn_search.cpp
               src/mean_yaw.cpp
               src/steer.cpp)

ament_target_dependencies(rrt_node OpenCV rclcpp sensor_msgs ackermann_msgs nav_msgs geometry_msgs tf2 visualization_msgs
                           f1tenth_msgs)
target_link_libraries(rrt_node geos flann PkgConfig::lz4 ${ARMADILLO_LIBRARIES} ${BLAS_LIBRARIES})

ament_target_dependencies(theta_star OpenCV)

ament_target_dependencies(speed_profile Armadillo BLAS)
target_link_libraries(speed_profile ${ARMADILLO_LIBRARIES} ${BLAS_LIBRARIES})

ament_target_dependencies(tests rclcpp)
target_link_libraries(tests geos nanoflann::nanoflann flann PkgConfig::lz4)

install(TARGETS
        rrt_node
        tests
        theta_star
        speed_profile
        curvature
        DESTINATION lib/${PROJECT_NAME})


ament_package()
