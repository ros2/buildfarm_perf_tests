cmake_minimum_required(VERSION 3.5)
project(buildfarm_perf_tests)

if(WIN32 OR APPLE OR ANDROID)
  message(STATUS "buildfarm_perf_tests is only supported on Linux, skipping...")
  return()
endif()

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

add_executable(node_spinning src/node_spinning.cpp)
ament_target_dependencies(node_spinning
  rclcpp
)

include_directories(include)

add_executable(system_metric_collector
  src/main_measurements.cpp
  src/linux_cpu_process_measurement.cpp
  src/linux_memory_process_measurement.cpp
  src/linux_cpu_system_measurement.cpp
  src/linux_memory_system_measurement.cpp
  src/utilities/utilities.cpp
)

target_link_libraries(system_metric_collector
  -lstdc++fs
)

install(TARGETS
  system_metric_collector
  DESTINATION
  lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

install(TARGETS
  node_spinning
  DESTINATION
  lib/${PROJECT_NAME}
)

install(DIRECTORY templates
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_config_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
