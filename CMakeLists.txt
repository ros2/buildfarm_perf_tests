cmake_minimum_required(VERSION 3.5)
project(buildfarm_perf_tests)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(system_metrics_collector REQUIRED)
find_package(metrics_statistics_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(hello_world_publisher src/hello_world_publisher.cpp)
ament_target_dependencies(hello_world_publisher
  rclcpp
  std_msgs
)

add_executable(hello_world_subscriber src/hello_world_subscriber.cpp)
ament_target_dependencies(hello_world_subscriber
  rclcpp
  std_msgs
)

add_executable(system_metric_collector src/system_metric_collector.cpp)
ament_target_dependencies(system_metric_collector
  system_metrics_collector
  metrics_statistics_msgs
  Boost
  rclcpp
)

install(TARGETS
  system_metric_collector
  hello_world_subscriber
  hello_world_publisher
  DESTINATION
  lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
