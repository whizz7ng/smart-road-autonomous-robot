cmake_minimum_required(VERSION 3.8)
project(traffic_light_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic — ROS 2 vendor 패키지 경유
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# ── 공유 라이브러리 빌드 ──────────────────────
add_library(TrafficLightPlugin SHARED
  src/TrafficLightPlugin.cc
)

target_link_libraries(TrafficLightPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::register
)

# ── install: colcon이 lib 경로를 잡을 수 있게 ──
install(TARGETS TrafficLightPlugin
  LIBRARY DESTINATION lib
)

ament_package()
