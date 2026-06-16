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
find_package(rcl_interfaces REQUIRED)
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
  rcl_interfaces
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
# PointCloudToPCD Component
# =====================================================
add_library(pointcloud_to_pcd_lib SHARED
  src/pointcloud_to_pcd.cpp
)

# Include directories and link libraries
target_include_directories(pointcloud_to_pcd_lib PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(pointcloud_to_pcd_lib
  ${PCL_LIBRARIES}
)

# Register the component
rclcpp_components_register_node(pointcloud_to_pcd_lib
  PLUGIN "perception_utils::PointCloudToPCD"
  EXECUTABLE pointcloud_to_pcd_node
)

ament_target_dependencies(pointcloud_to_pcd_lib ${DEPENDENCIES})


# =====================================================
# PCDToPointcloud Component
# =====================================================
add_library(pcd_to_pointcloud_lib SHARED
  src/pcd_to_pointcloud.cpp
)

# Include directories and link libraries
target_include_directories(pcd_to_pointcloud_lib PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcd_to_pointcloud_lib
  ${PCL_LIBRARIES}
)

# Register the component
rclcpp_components_register_node(pcd_to_pointcloud_lib
  PLUGIN "perception_utils::PCDToPointCloud"
  EXECUTABLE pcd_to_pointcloud_node
)

ament_target_dependencies(pcd_to_pointcloud_lib ${DEPENDENCIES})


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
# LaserscanMerger Executable
# =====================================================
add_executable(laserscan_multi_merger
  src/laserscan_multi_merger.cpp
)

target_include_directories(laserscan_multi_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(laserscan_multi_merger
  ${PCL_LIBRARIES}
)

ament_target_dependencies(laserscan_multi_merger ${DEPENDENCIES})

# =====================================================
# Install Targets and Directories
# =====================================================
install(
  TARGETS
    pointcloud_to_pcd_lib
    pcd_to_pointcloud_lib
    pointcloud_merger_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    pointcloud_to_pcd_node
    pcd_to_pointcloud_node
    pointcloud_merger
    laserscan_multi_merger
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
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
