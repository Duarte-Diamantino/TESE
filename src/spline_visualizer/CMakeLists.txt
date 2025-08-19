cmake_minimum_required(VERSION 3.8)
project(spline_visualizer)

# Usa C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings extra para GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) Encontra as dependências
find_package(ament_cmake REQUIRED)               # sistema de build ROS 2 :contentReference[oaicite:0]{index=0}
find_package(rclcpp REQUIRED)                    # criação de nós C++ :contentReference[oaicite:1]{index=1}
find_package(visualization_msgs REQUIRED)        # Marker para RViz2 :contentReference[oaicite:2]{index=2}
find_package(nav_msgs REQUIRED)                  # TOPICO PARA O MPC
find_package(geometry_msgs REQUIRED)             # Point para montar a spline :contentReference[oaicite:3]{index=3}

# 2) Define o executável
add_executable(spline_visualizer src/spline_visualizer.cpp)
ament_target_dependencies(
  spline_visualizer
  rclcpp
  visualization_msgs
  nav_msgs
  geometry_msgs
)

# 3) Instala para que 'ros2 run' o encontre
install(
  TARGETS spline_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Finaliza a configuração do pacote
ament_package()
