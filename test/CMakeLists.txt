#
# Global performance test settings
#

set(PERF_TEST_RUNTIME 30
  CACHE STRING "Duration for each performance test run")
set(PERF_TEST_TOPIC_PUB_SUB Array60k
  CACHE STRING "perf_test topic used by pub/stub tests")
set(PERF_TEST_TOPICS
  Array1k
  Array4k
  Array16k
  Array32k
  Array60k
  PointCloud512k
  Array1m
  Array2m
  CACHE STRING "List of perf_test topics to run tests with"
)

#
# Standalone (non-ROS) performance test settings
#

set(PERF_TEST_COMM_TYPE_rmw_cyclonedds_cpp "CycloneDDS"
  CACHE STRING "Standalone COMM type for rmw_cyclonedds_cpp")
set(PERF_TEST_COMM_TYPE_rmw_fastrtps_cpp "FastRTPS"
  CACHE STRING "Standalone COMM type for rmw_fastrtps_cpp")

#
# Special environment variable settings
#

# Fast-RTPS RMWs need some special env vars for sync mode
get_filename_component(FASTRTPS_DEFAULT_PROFILES_FILE
  "${CMAKE_CURRENT_SOURCE_DIR}/DEFAULT_FASTRTPS_PROFILES.xml" ABSOLUTE)
set(PERF_TEST_ENV_rmw_fastrtps_cpp_sync
  "RMW_FASTRTPS_USE_QOS_FROM_XML=1"
  "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
)
set(PERF_TEST_ENV_rmw_fastrtps_dynamic_cpp_sync
  "RMW_FASTRTPS_USE_QOS_FROM_XML=1"
  "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
)

# ...and some EXTRA special env vars for multi-process sync mode
get_filename_component(FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE
  "${CMAKE_CURRENT_SOURCE_DIR}/DEFAULT_FASTRTPS_PROFILES_TWO_PROCESSES.xml" ABSOLUTE)
set(PERF_TEST_ENV_two_processes_rmw_fastrtps_cpp_sync
  "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE}"
)
set(PERF_TEST_ENV_two_processes_rmw_fastrtps_dynamic_cpp_sync
  "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE}"
)
set(PERF_TEST_ENV_pub_sub_rmw_fastrtps_cpp_sync
  "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE}"
)
set(PERF_TEST_ENV_pub_sub_rmw_fastrtps_dynamic_cpp_sync
  "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE}"
)

#
# Skipped test settings
#

# Some DDS/RMWs don't support async/sync
set(PERF_TEST_SKIP_CycloneDDS_async ON
  CACHE BOOL "Skip CycloneDDS async tests")
set(PERF_TEST_SKIP_rmw_cyclonedds_cpp_async ON
  CACHE BOOL "Skip rmw_cyclonedds_cpp async tests")
set(PERF_TEST_SKIP_rmw_opensplice_cpp_sync ON
  CACHE BOOL "Skip rmw_opensplice_cpp sync tests")
set(PERF_TEST_SKIP_rmw_connext_cpp_sync ON
  CACHE BOOL "Skip rmw_connext_cpp sync tests")

# Fast-RTPS ros2-eloquent branch (1.9.3 + bugfixes) does not support synchronous
# sending of fragments. This means that in that branch, topics PointCloud512k,
# Array1m, and Array2m cannot be sent synchronously in ROS 2 Eloquent (either
# from standalone Fast-RTPS, or from rmw_fastrtps_*)
# https://github.com/ros2/buildfarm_perf_tests/pull/26#discussion_r393229707
set(FASTRTPS_SYNC_SKIP_ROS_DISTROS
  "dashing"
  "eloquent"
)
if("$ENV{ROS_DISTRO}" IN_LIST FASTRTPS_SYNC_SKIP_ROS_DISTROS)
  foreach(skip_rmw rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp FastRTPS)
    foreach(skip_topic Array1m Array2m PointCloud512k)
      set(PERF_TEST_SKIP_${skip_rmw}_sync_${skip_topic} ON
        CACHE BOOL "Skip ${skip_rmw} sync ${skip_topic} tests")
    endforeach()
  endforeach()
endif()

# TODO (ahcorde): perf_test is not working with CycloneDDS when
# processes are splitted in two
# what(): Round trip mode is not implemented for Cyclone DDS!
set(PERF_TEST_SKIP_two_process_CycloneDDS_async ON
  CACHE BOOL "Skip two-process CycloneDDS async tests")
set(PERF_TEST_SKIP_two_process_CycloneDDS_sync ON
  CACHE BOOL "Skip two-process CycloneDDS sync tests")

#
# Add performance tests
#

find_package(ament_cmake_flake8 REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

ament_flake8(TESTNAME "flake8_generated"
  ${CMAKE_CURRENT_BINARY_DIR}/test
)

include(add_performance_tests.cmake)
call_for_each_rmw_implementation(add_performance_tests)
