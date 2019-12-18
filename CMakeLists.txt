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
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(node_spinning src/node_spinning.cpp)
ament_target_dependencies(node_spinning
  rclcpp
)

# add_executable(system_metric_collector src/system_metric_collector.cpp)
# ament_target_dependencies(system_metric_collector
#   system_metrics_collector
#   metrics_statistics_msgs
#   Boost
#   rclcpp
#   rcpputils
#   rcutils
# )

include_directories(include)

add_executable(system_metric_collector
  src/main_measurements.cpp
  src/linux_cpu_measurement.cpp
  src/linux_memory_measurement.cpp
  src/utilities/utilities.cpp
)
ament_target_dependencies(system_metric_collector
  Boost
)

target_link_libraries(system_metric_collector
  -lstdc++fs
)

install(TARGETS
  system_metric_collector
  node_spinning
  DESTINATION
  lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
