set(PERF_TEST_MAX_RUNTIME 30
  CACHE STRING "Duration for each performance test run")

find_package(launch_testing_ament_cmake REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

set(COMM_TYPE_rmw_cyclonedds_cpp "CycloneDDS")
set(COMM_TYPE_rmw_fastrtps_cpp "FastRTPS")

set(TOPICS
  Array1k
  Array4k
  Array16k
  Array32k
  Array60k
  PointCloud512k
  Array1m
  Array2m
)

function(add_performance_test TEST_NAME COMM RMW_IMPLEMENTATION SYNC_MODE)

  if(${SYNC_MODE} STREQUAL "async" AND ${RMW_IMPLEMENTATION} STREQUAL "rmw_cyclonedds_cpp")
    message(STATUS "No async mode in RMW_IMPLEMENTATION: rmw_cyclonedds_cpp")
    return()
  endif()

  if(${SYNC_MODE} STREQUAL "sync" AND ${RMW_IMPLEMENTATION} STREQUAL "rmw_opensplice_cpp")
    message(STATUS "No sync mode in RMW_IMPLEMENTATION: rmw_opensplice_cpp")
    return()
  endif()

  if(${SYNC_MODE} STREQUAL "sync" AND ${RMW_IMPLEMENTATION} STREQUAL "rmw_connext_cpp")
    message(STATUS "No sync mode in RMW_IMPLEMENTATION: rmw_connext_cpp")
    return()
  endif()

  # Enviroment variables to configure rmw_fastrtps_cpp in sync or async
  if(${SYNC_MODE} STREQUAL "sync" AND
    (${RMW_IMPLEMENTATION} STREQUAL "rmw_fastrtps_cpp" OR ${RMW_IMPLEMENTATION} STREQUAL "rmw_fastrtps_dynamic_cpp"))
      set(RMW_FASTRTPS_USE_QOS_FROM_XML 1)
      get_filename_component(
          FASTRTPS_DEFAULT_PROFILES_FILE
          "${CMAKE_CURRENT_SOURCE_DIR}/DEFAULT_FASTRTPS_PROFILES.xml"
          ABSOLUTE
      )
      get_filename_component(
          FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE
          "${CMAKE_CURRENT_SOURCE_DIR}/DEFAULT_FASTRTPS_PROFILES_TWO_PROCESSES.xml"
          ABSOLUTE
      )
  else()
    set(RMW_FASTRTPS_USE_QOS_FROM_XML 0)
    set(FASTRTPS_DEFAULT_PROFILES_FILE "")
    set(FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE "")
  endif()

  foreach(topic ${TOPICS})

    set(PERF_TEST_TOPIC ${topic})

    # Fast-RTPS ros2-eloquent branch (1.9.3 + bugfixes) does not support synchronous
    # sending of fragments. This means that in that branch, topics PointCloud512k,
    # Array1m, and Array2m cannot be sent synchronously in ROS 2 Eloquent (either
    # from standalone Fast-RTPS, or from rmw_fastrtps_*)
    # https://github.com/ros2/buildfarm_perf_tests/pull/26#discussion_r393229707
    set(SKIP_TEST "")
    if(($ENV{ROS_DISTRO} STREQUAL "eloquent" OR $ENV{ROS_DISTRO} STREQUAL "dashing") AND
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "rmw_fastrtps_cpp_sync_Array2m" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "rmw_fastrtps_cpp_sync_Array1m" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "rmw_fastrtps_cpp_sync_PointCloud512k" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "rmw_fastrtps_dynamic_cpp_sync_Array2m" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "rmw_fastrtps_dynamic_cpp_sync_Array1m" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "rmw_fastrtps_dynamic_cpp_sync_PointCloud512k" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "FastRTPS_sync_Array2m" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "FastRTPS_sync_Array1m" OR
        ${TEST_NAME}_${PERF_TEST_TOPIC} STREQUAL "FastRTPS_sync_PointCloud512k"
      )
      set(SKIP_TEST "SKIP_TEST")
    endif()

    set(NUMBER_PROCESS "1")
    get_filename_component(
      PERFORMANCE_REPORT_PNG
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_results_${TEST_NAME}_${PERF_TEST_TOPIC}.png"
      ABSOLUTE
    )
    get_filename_component(
      PERFORMANCE_REPORT_CSV
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_results_${TEST_NAME}_${PERF_TEST_TOPIC}.csv"
      ABSOLUTE
    )
    configure_file(
      test_performance.py.in
      ${CMAKE_CURRENT_BINARY_DIR}/test_performance_${TEST_NAME}_${PERF_TEST_TOPIC}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${TEST_NAME}_${PERF_TEST_TOPIC}.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_performance_${TEST_NAME}_${PERF_TEST_TOPIC}.py.configure"
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${TEST_NAME}_${PERF_TEST_TOPIC}.py"
      TARGET test_performance_${TEST_NAME}_${PERF_TEST_TOPIC}
      ENV
      "PERFORMANCE_REPORT_PNG=${PERFORMANCE_REPORT_PNG}"
      "PERFORMANCE_REPORT_CSV=${PERFORMANCE_REPORT_CSV}"
      "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
      "RMW_FASTRTPS_USE_QOS_FROM_XML=${RMW_FASTRTPS_USE_QOS_FROM_XML}"
      "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
      ${SKIP_TEST}
    )

    set(NUMBER_PROCESS "2")
    # TODO (ahcorde): perf_test is not working with CycloneDDS when
    # processes are splitted in two
    if(${COMM} STREQUAL "CycloneDDS")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    get_filename_component(
      PERFORMANCE_TWO_PROCESS_REPORT_PNG
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_two_process_results_${TEST_NAME}_${PERF_TEST_TOPIC}.png"
      ABSOLUTE
    )
    get_filename_component(
      PERFORMANCE_TWO_PROCESS_REPORT_CSV
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_two_process_results_${TEST_NAME}_${PERF_TEST_TOPIC}.csv"
      ABSOLUTE
    )
    configure_file(
      test_performance.py.in
      ${CMAKE_CURRENT_BINARY_DIR}/test_performance_two_process_${TEST_NAME}_${PERF_TEST_TOPIC}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_two_process_${TEST_NAME}_${PERF_TEST_TOPIC}.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_performance_two_process_${TEST_NAME}_${PERF_TEST_TOPIC}.py.configure"
    )

    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_two_process_${TEST_NAME}_${PERF_TEST_TOPIC}.py"
      TARGET test_performance_two_process_${TEST_NAME}_${PERF_TEST_TOPIC}
      ENV
      "PERFORMANCE_REPORT_PNG=${PERFORMANCE_TWO_PROCESS_REPORT_PNG}"
      "PERFORMANCE_REPORT_CSV=${PERFORMANCE_TWO_PROCESS_REPORT_CSV}"
      "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
      "RMW_FASTRTPS_USE_QOS_FROM_XML=${RMW_FASTRTPS_USE_QOS_FROM_XML}"
      "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE}"
      ${SKIP_TEST}
    )
  endforeach()

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
    "RMW_FASTRTPS_USE_QOS_FROM_XML=${RMW_FASTRTPS_USE_QOS_FROM_XML}"
    "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
    TIMEOUT 120
  )
  set(PUBSUB_TIMEOUT "30")

  get_available_rmw_implementations(_rmw_implementations)

  foreach(rmw_impl ${_rmw_implementations})

    # TODO (ahcorde): perf_test is not working with CycloneDDS when
    #  processes are splitted in two
    # what(): Round tip mode is not implemented for Cyclone DDS
    set(SKIP_TEST "")
    if(${COMM} STREQUAL "CycloneDDS")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    set(COMM_PUB ${RMW_IMPLEMENTATION})
    set(COMM_SUB ${rmw_impl})
    set(TEST_NAME_PUB_SUB ${TEST_NAME}_${rmw_impl}_${COMM})

    get_filename_component(
      PERFORMANCE_OVERHEAD_CSV
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_test_results_${TEST_NAME_PUB_SUB}.csv"
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
      "RMW_FASTRTPS_USE_QOS_FROM_XML=${RMW_FASTRTPS_USE_QOS_FROM_XML}"
      "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_TWO_PROCESSES_FILE}"
      TIMEOUT 120
      ${SKIP_TEST}
    )
  endforeach()
endfunction()

macro(try_add_performance_test)
  add_performance_test(
    "${rmw_implementation}_async"
    "ROS2"
    ${rmw_implementation}
    "async"
  )

  add_performance_test(
    "${rmw_implementation}_sync"
    "ROS2"
    ${rmw_implementation}
    "sync"
  )

  if(COMM_TYPE_${rmw_implementation})
    add_performance_test(
      "${COMM_TYPE_${rmw_implementation}}_async"
      ${COMM_TYPE_${rmw_implementation}}
      ${rmw_implementation}
      "async"
    )
    add_performance_test(
      "${COMM_TYPE_${rmw_implementation}}_sync"
      ${COMM_TYPE_${rmw_implementation}}
      ${rmw_implementation}
      "sync"
    )
  else()
    message(STATUS "No standalone DDS support for RMW: ${rmw_implementation}")
  endif()
endmacro()

call_for_each_rmw_implementation(try_add_performance_test)
