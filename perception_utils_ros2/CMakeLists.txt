cmake_minimum_required(VERSION 3.8)
project(perception_utils_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common filters)

# Dependencies list (to avoid repeating them)
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  Eigen3
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  PCL
)

# =====================================================
# CombinedPointCloudToPCD Component
# =====================================================
add_library(combined_pointcloud_to_pcd_lib SHARED
  src/combined_pointcloud_to_pcd.cpp
)

# Include directories and link libraries
target_include_directories(combined_pointcloud_to_pcd_lib PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(combined_pointcloud_to_pcd_lib
  ${PCL_LIBRARIES}
)

# Register the component
rclcpp_components_register_node(combined_pointcloud_to_pcd_lib
  PLUGIN "perception_utils::CombinedPointCloudToPCD"
  EXECUTABLE combined_pointcloud_to_pcd_node
)

ament_target_dependencies(combined_pointcloud_to_pcd_lib ${DEPENDENCIES})

# =====================================================
# PointCloudMerger Component
# =====================================================
add_library(pointcloud_merger_lib SHARED
  src/pointcloud_merger.cpp
)

target_include_directories(pointcloud_merger_lib PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_merger_lib
  ${PCL_LIBRARIES}
)

# Register the component
rclcpp_components_register_node(pointcloud_merger_lib
  PLUGIN "perception_utils::PointCloudMerger"
  EXECUTABLE pointcloud_merger
)

ament_target_dependencies(pointcloud_merger_lib ${DEPENDENCIES})

# =====================================================
# Install Targets and Directories
# =====================================================
install(
  TARGETS combined_pointcloud_to_pcd_lib pointcloud_merger_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# =====================================================
# Testing Configuration
# =====================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
