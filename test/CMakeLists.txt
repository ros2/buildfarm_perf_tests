set(PERF_TEST_MAX_RUNTIME 30
  CACHE STRING "Duration for each performance test run")
set(PERF_TEST_TOPIC "Array1k"
  CACHE STRING "Performance test topic")

find_package(launch_testing_ament_cmake REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

set(COMM_TYPE_rmw_cyclonedds_cpp "CycloneDDS")
set(COMM_TYPE_rmw_fastrtps_cpp "FastRTPS")

function(add_performance_test TEST_NAME COMM RMW_IMPLEMENTATION)
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

  set(PUBSUB_TIMEOUT "30")

  get_available_rmw_implementations(_rmw_implementations)

  foreach(rmw_impl ${_rmw_implementations})

    set(COMM_PUB ${COMM})
    set(COMM_SUB ${rmw_impl})
    set(TEST_NAME_PUB_SUB ${COMM}_${RMW_IMPLEMENTATION}_${rmw_impl})

    get_filename_component(
      PERFORMANCE_OVERHEAD_CSV
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_test_results.csv"
      ABSOLUTE
    )
    get_filename_component(
      PERFORMANCE_OVERHEAD_PNG
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_test_results"
      ABSOLUTE
    )

    configure_file(
      test_pub_sub.py.in
      ${CMAKE_CURRENT_BINARY_DIR}/test_pub_sub_${TEST_NAME_PUB_SUB}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_pub_sub_${TEST_NAME_PUB_SUB}.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_pub_sub_${TEST_NAME_PUB_SUB}.py.configure"
    )

    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_pub_sub_${TEST_NAME_PUB_SUB}.py"
      TARGET test_pub_sub_${TEST_NAME_PUB_SUB}
      ENV
      "PERFORMANCE_OVERHEAD_CSV=${PERFORMANCE_OVERHEAD_CSV}"
      "PERFORMANCE_OVERHEAD_PNG=${PERFORMANCE_OVERHEAD_PNG}"
      "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
      TIMEOUT 120
    )
  endforeach()
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
