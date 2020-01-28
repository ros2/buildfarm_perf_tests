set(PERF_TEST_MAX_RUNTIME 30
  CACHE STRING "Duration for each performance test run")
set(PERF_TEST_TOPIC "Array1k"
  CACHE STRING "Performance test topic")

find_package(launch_testing_ament_cmake REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

set(COMM_TYPE_rmw_cyclonedds_cpp "CycloneDDS")
set(COMM_TYPE_rmw_fastrtps_cpp "FastRTPS")

function(add_performance_test TEST_NAME COMM RMW_IMPLEMENTATION)
  set(NUMBER_PROCESS "1")

  get_filename_component(
    PERFORMANCE_REPORT_PNG
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_results_${TEST_NAME}.png"
    ABSOLUTE
  )
  get_filename_component(
    PERFORMANCE_REPORT_CSV
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_results_${TEST_NAME}.csv"
    ABSOLUTE
  )
  configure_file(
    test_performance.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/test_performance_${TEST_NAME}.py.configure
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${TEST_NAME}.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_performance_${TEST_NAME}.py.configure"
  )
  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${TEST_NAME}.py"
    TARGET test_performance_${TEST_NAME}
    ENV
    "PERFORMANCE_REPORT_PNG=${PERFORMANCE_REPORT_PNG}"
    "PERFORMANCE_REPORT_CSV=${PERFORMANCE_REPORT_CSV}"
    "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  )

  set(NUMBER_PROCESS "2")
  # TODO (ahcorde): perf_test is not working with CycloneDDS or FastRTPS when
  # processes are splitted in two
  set(SKIP_TEST "")
  if(${COMM} STREQUAL "CycloneDDS" OR ${COMM} STREQUAL "FastRTPS")
    set(SKIP_TEST "SKIP_TEST")
  endif()

  get_filename_component(
    PERFORMANCE_TWO_PROCESS_REPORT_PNG
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_two_process_results_${TEST_NAME}.png"
    ABSOLUTE
  )
  get_filename_component(
    PERFORMANCE_TWO_PROCESS_REPORT_CSV
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_two_process_results_${TEST_NAME}.csv"
    ABSOLUTE
  )
  configure_file(
    test_performance.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/test_performance_two_process_${TEST_NAME}.py.configure
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_two_process_${TEST_NAME}.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_performance_two_process_${TEST_NAME}.py.configure"
  )

  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_two_process_${TEST_NAME}.py"
    TARGET test_performance_two_process_${TEST_NAME}
    ENV
    "PERFORMANCE_REPORT_PNG=${PERFORMANCE_TWO_PROCESS_REPORT_PNG}"
    "PERFORMANCE_REPORT_CSV=${PERFORMANCE_TWO_PROCESS_REPORT_CSV}"
    "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
    ${SKIP_TEST}
  )

  set(NODE_SPINNING_TIMEOUT "30")
  get_filename_component(
    PERFORMANCE_OVERHEAD_NODE_CSV
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_node_test_results_${TEST_NAME}.csv"
    ABSOLUTE
  )
  get_filename_component(
    PERFORMANCE_OVERHEAD_NODE_PNG
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_node_test_results_${TEST_NAME}.png"
    ABSOLUTE
  )

  configure_file(
    test_spinning.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/test_spinning_${TEST_NAME}.py.configure
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_spinning_${TEST_NAME}.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_spinning_${TEST_NAME}.py.configure"
  )

  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/test_spinning_${TEST_NAME}.py"
    TARGET test_spinning_${TEST_NAME}
    ENV
    "PERFORMANCE_OVERHEAD_NODE_CSV=${PERFORMANCE_OVERHEAD_NODE_CSV}"
    "PERFORMANCE_OVERHEAD_NODE_PNG=${PERFORMANCE_OVERHEAD_NODE_PNG}"
    "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
    TIMEOUT 120
  )
endfunction()

macro(try_add_performance_test)
  add_performance_test(
    ${rmw_implementation}
    "ROS2"
    ${rmw_implementation}
  )
  if(COMM_TYPE_${rmw_implementation})
    add_performance_test(
      ${COMM_TYPE_${rmw_implementation}}
      ${COMM_TYPE_${rmw_implementation}}
      ${rmw_implementation}
    )
  else()
    message(STATUS "No standalone DDS support for RMW: ${rmw_implementation}")
  endif()
endmacro()

call_for_each_rmw_implementation(try_add_performance_test)
