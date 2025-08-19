cmake_minimum_required(VERSION 3.8)
project(dwa_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(dwa_follower_node src/dwa_follower.cpp)

ament_target_dependencies(dwa_follower_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

add_executable(dwa_spline_follower
  src/dwa_spline_follower.cpp
)
ament_target_dependencies(dwa_spline_follower
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Novo nó: Global DWA follower
add_executable(dwa_global_follower
  src/dwa_global_follower.cpp
)
ament_target_dependencies(dwa_global_follower
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  std_msgs
)

# Instalação
install(
  TARGETS
    dwa_follower_node
    dwa_spline_follower
    dwa_global_follower
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip license/header checks until you've added them
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
