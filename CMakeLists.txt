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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(node_spinning src/node_spinning.cpp)
ament_target_dependencies(
  node_spinning
  "rclcpp"
)

install(TARGETS node_spinning
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

option(ENABLE_SYSTEM_METRIC_COLLECTOR
  "Enables the system metric collector and tests which use it" ${UNIX})
if(ENABLE_SYSTEM_METRIC_COLLECTOR)
  add_executable(system_metric_collector
    src/linux_cpu_process_measurement.cpp
    src/linux_cpu_system_measurement.cpp
    src/linux_memory_process_measurement.cpp
    src/linux_memory_system_measurement.cpp
    src/main_measurements.cpp
    src/utilities/utilities.cpp
  )
  target_include_directories(system_metric_collector PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(system_metric_collector "stdc++fs")

  install(TARGETS system_metric_collector
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY templates
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/generate_config_yaml.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
