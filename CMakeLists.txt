cmake_policy(SET CMP0074 NEW)
cmake_minimum_required(VERSION 3.8)

project(relocalization_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration features)
find_package(teaserpp REQUIRED)

find_library(TEASER_REGISTRATION teaser_registration PATHS /usr/local/lib)
find_library(TEASER_IO teaser_io PATHS /usr/local/lib)

include_directories(include ${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(
  relocalization_node
  src/main.cpp src/relocalization_node.cpp src/scan_processor.cpp
  src/global_registration.cpp src/local_refinement.cpp)

ament_target_dependencies(
  relocalization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions)

target_link_libraries(relocalization_node ${PCL_LIBRARIES}
                      ${TEASER_REGISTRATION} ${TEASER_IO})
install(TARGETS relocalization_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
