cmake_minimum_required(VERSION 3.8)
project(obstacle_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- aqui declaramos o executável e as deps ----------------------
add_executable(obstacle_publisher_node
  src/obstacle_publisher.cpp
)

ament_target_dependencies(obstacle_publisher_node
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
)

# --- instalação do executável ------------------------------------
install(TARGETS obstacle_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- testes (mantém se precisares) --------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# exporta o pacote
ament_package()
