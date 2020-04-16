# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

function(add_performance_test TEST_NAME COMM RMW_IMPLEMENTATION SYNC_MODE)

  #
  # Setup for performance sub-tests
  #

  set(TEST_NAME_SYNC "${TEST_NAME}_${SYNC_MODE}")
  set(SKIP_TEST "")
  if(PERF_TEST_SKIP_${TEST_NAME_SYNC})
    message(STATUS "Skipping performance tests: ${TEST_NAME_SYNC}")
    set(SKIP_TEST "SKIP_TEST")
  endif()

  set(TEST_ENV "")
  if(COMM STREQUAL "ROS2")
    list(APPEND TEST_ENV "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}")
  endif()
  list(APPEND TEST_ENV ${PERF_TEST_ENV_${TEST_NAME_SYNC}})

  #
  # Setup for per-topic tests
  #

  foreach(PERF_TEST_TOPIC ${PERF_TEST_TOPICS})
    # Start with single-process tests

    set(TEST_NAME_SYNC_TOPIC "${TEST_NAME_SYNC}_${PERF_TEST_TOPIC}")
    set(SKIP_TEST_TOPIC "${SKIP_TEST}")
    if(NOT SKIP_TEST_TOPIC)
      if(PERF_TEST_SKIP_${TEST_NAME_SYNC_TOPIC} OR PERF_TEST_SKIP_apex_ai_suite)
        message(STATUS "Skipping performance tests: ${TEST_NAME_SYNC_TOPIC}")
        set(SKIP_TEST_TOPIC "SKIP_TEST")
      endif()
    endif()
    set(TEST_ENV_TOPIC ${TEST_ENV})
    list(APPEND TEST_ENV_TOPIC ${PERF_TEST_ENV_${TEST_NAME_SYNC_TOPIC}})

    set(NUMBER_PROCESS "1")

    get_filename_component(
      PERFORMANCE_REPORT_PNG
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_results_${TEST_NAME_SYNC_TOPIC}.png"
      ABSOLUTE
    )
    get_filename_component(
      PERFORMANCE_REPORT_CSV
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_results_${TEST_NAME_SYNC_TOPIC}.csv"
      ABSOLUTE
    )
    list(APPEND TEST_ENV_TOPIC
      "PERFORMANCE_REPORT_PNG=${PERFORMANCE_REPORT_PNG}"
      "PERFORMANCE_REPORT_CSV=${PERFORMANCE_REPORT_CSV}"
    )

    configure_file(
      test_performance.py.in
      ${CMAKE_CURRENT_BINARY_DIR}/test_performance_${TEST_NAME_SYNC_TOPIC}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${TEST_NAME_SYNC_TOPIC}.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_performance_${TEST_NAME_SYNC_TOPIC}.py.configure"
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${TEST_NAME_SYNC_TOPIC}.py"
      TARGET test_performance_${TEST_NAME_SYNC_TOPIC}
      ENV ${TEST_ENV_TOPIC}
      ${SKIP_TEST_TOPIC}
    )
    set_tests_properties(test_performance_${TEST_NAME_SYNC_TOPIC} PROPERTIES
      RUN_SERIAL TRUE
      LABELS "performance"
    )

    # Inherit settings from single-process to run two-process tests

    if(NOT SKIP_TEST_TOPIC)
      if(PERF_TEST_SKIP_two_process_${TEST_NAME_SYNC_TOPIC} OR
          PERF_TEST_SKIP_two_process_${TEST_NAME_SYNC})
        message(STATUS "Skipping performance tests: two_process_${TEST_NAME_SYNC_TOPIC}")
        set(SKIP_TEST_TOPIC "SKIP_TEST")
      endif()
    endif()
    list(APPEND TEST_ENV_TOPIC ${PERF_TEST_ENV_two_process_${TEST_NAME_SYNC_TOPIC}})

    set(NUMBER_PROCESS "2")

    get_filename_component(
      PERFORMANCE_REPORT_PNG
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_two_process_results_${TEST_NAME_SYNC_TOPIC}.png"
      ABSOLUTE
    )
    get_filename_component(
      PERFORMANCE_REPORT_CSV
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/performance_test_two_process_results_${TEST_NAME_SYNC_TOPIC}.csv"
      ABSOLUTE
    )
    list(APPEND TEST_ENV_TOPIC
      "PERFORMANCE_REPORT_PNG=${PERFORMANCE_REPORT_PNG}"
      "PERFORMANCE_REPORT_CSV=${PERFORMANCE_REPORT_CSV}"
    )

    configure_file(
      test_performance.py.in
      ${CMAKE_CURRENT_BINARY_DIR}/test_performance_two_process_${TEST_NAME_SYNC_TOPIC}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_two_process_${TEST_NAME_SYNC_TOPIC}.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_performance_two_process_${TEST_NAME_SYNC_TOPIC}.py.configure"
    )

    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_two_process_${TEST_NAME_SYNC_TOPIC}.py"
      TARGET test_performance_two_process_${TEST_NAME_SYNC_TOPIC}
      ENV ${TEST_ENV_TOPIC}
      ${SKIP_TEST_TOPIC}
    )
    set_tests_properties(test_performance_two_process_${TEST_NAME_SYNC_TOPIC} PROPERTIES
      RUN_SERIAL TRUE
      LABELS "performance"
    )
  endforeach()

  #
  # All remaining tests only apply to RMWs
  #

  if(NOT "${COMM}" STREQUAL "ROS2")
    # Since standalone DDS can never run correctly, exclude instead of skipping the tests.
    return()
  endif()

  #
  # All remaining tests require the system_metric_collector
  #

  if(NOT ENABLE_SYSTEM_METRIC_COLLECTOR)
    set(SKIP_TEST "SKIP_TEST")
    message(STATUS "Skipping performance tests: ${TEST_NAME_SYNC} (no system_metric_collector)")
  endif()

  #
  # Setup for spinning tests
  #

  set(SKIP_TEST_SPINNING "${SKIP_TEST}")
  if(NOT SKIP_TEST_SPINNING)
    if(PERF_TEST_SKIP_spinning_${TEST_NAME_SYNC} OR
        PERF_TEST_SKIP_spinning OR NOT ENABLE_SYSTEM_METRIC_COLLECTOR)
      message(STATUS "Skipping performance tests: spinning_${TEST_NAME_SYNC}")
      set(SKIP_TEST_SPINNING "SKIP_TEST")
    endif()
  endif()
  set(TEST_ENV_SPINNING ${TEST_ENV})
  list(APPEND TEST_ENV_SPINNING ${PERF_TEST_ENV_spinning_${TEST_NAME_SYNC}})

  set(NODE_SPINNING_TIMEOUT "30")

  get_filename_component(
    PERFORMANCE_OVERHEAD_NODE_CSV
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_node_test_results_${TEST_NAME_SYNC}.csv"
    ABSOLUTE
  )
  get_filename_component(
    PERFORMANCE_OVERHEAD_NODE_PNG
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_node_test_results_${TEST_NAME_SYNC}.png"
    ABSOLUTE
  )
  list(APPEND TEST_ENV_SPINNING
    "PERFORMANCE_OVERHEAD_NODE_CSV=${PERFORMANCE_OVERHEAD_NODE_CSV}"
    "PERFORMANCE_OVERHEAD_NODE_PNG=${PERFORMANCE_OVERHEAD_NODE_PNG}"
  )

  configure_file(
    test_spinning.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/test_spinning_${TEST_NAME_SYNC}.py.configure
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_spinning_${TEST_NAME_SYNC}.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_spinning_${TEST_NAME_SYNC}.py.configure"
  )

  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/test_spinning_${TEST_NAME_SYNC}.py"
    TARGET test_spinning_${TEST_NAME_SYNC}
    ENV ${TEST_ENV_SPINNING}
    TIMEOUT 120
    ${SKIP_TEST_SPINNING}
  )
  set_tests_properties(test_spinning_${TEST_NAME_SYNC} PROPERTIES
    RUN_SERIAL TRUE
    LABELS "performance"
  )

  #
  # Setup for cross-vendor tests
  #

  set(PUBSUB_TIMEOUT "30")
  set(PERF_TEST_TOPIC ${PERF_TEST_TOPIC_PUB_SUB})
  get_available_rmw_implementations(_rmw_implementations)
  foreach(RMW_IMPLEMENTATION_SUB ${_rmw_implementations})
    # TODO: Why is ${COMM} on the end here? It's always ROS2.
    set(TEST_NAME_PUB_SUB "${TEST_NAME_SYNC}_${RMW_IMPLEMENTATION_SUB}_${COMM}")
    set(SKIP_TEST_PUB_SUB "${SKIP_TEST}")
    if(NOT SKIP_TEST_PUB_SUB)
      if(PERF_TEST_SKIP_${TEST_NAME_PUB_SUB} OR PERF_TEST_SKIP_pub_sub_${TEST_NAME_SYNC} OR
          PERF_TEST_SKIP_pub_sub OR PERF_TEST_SKIP_apex_ai_suite OR NOT ENABLE_SYSTEM_METRIC_COLLECTOR)
        message(STATUS "Skipping performance tests: ${TEST_NAME_PUB_SUB}")
        set(SKIP_TEST_PUB_SUB "SKIP_TEST")
      endif()
    endif()
    set(TEST_ENV_PUB_SUB ${TEST_ENV})
    list(APPEND TEST_ENV_PUB_SUB ${PERF_TEST_ENV_pub_sub_${TEST_NAME_SYNC}})
    list(APPEND TEST_ENV_PUB_SUB ${PERF_TEST_ENV_${TEST_NAME_PUB_SUB}})

    get_filename_component(
      PERFORMANCE_OVERHEAD_CSV
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_test_results_${TEST_NAME_PUB_SUB}.csv"
      ABSOLUTE
    )
    # TODO: Why doesn't this file name have a suffix?
    get_filename_component(
      PERFORMANCE_OVERHEAD_PNG
      "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/overhead_test_results"
      ABSOLUTE
    )
    list(APPEND TEST_ENV_PUB_SUB
      "PERFORMANCE_OVERHEAD_CSV=${PERFORMANCE_OVERHEAD_CSV}"
      "PERFORMANCE_OVERHEAD_PNG=${PERFORMANCE_OVERHEAD_PNG}"
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
      ENV ${TEST_ENV_PUB_SUB}
      TIMEOUT 120
      ${SKIP_TEST_PUB_SUB}
    )
    set_tests_properties(test_pub_sub_${TEST_NAME_PUB_SUB} PROPERTIES
      RUN_SERIAL TRUE
      LABELS "performance"
    )
  endforeach()

endfunction()

macro(add_performance_tests)
  foreach(PERF_TEST_SYNC "async" "sync")
    add_performance_test(
      ${rmw_implementation}
      "ROS2"
      ${rmw_implementation}
      ${PERF_TEST_SYNC}
    )
  endforeach()

  if(PERF_TEST_COMM_TYPE_${rmw_implementation})
    foreach(PERF_TEST_SYNC "async" "sync")
      add_performance_test(
        ${PERF_TEST_COMM_TYPE_${rmw_implementation}}
        ${PERF_TEST_COMM_TYPE_${rmw_implementation}}
        ${rmw_implementation}
        ${PERF_TEST_SYNC}
      )
    endforeach()
  else()
    message(STATUS "No standalone DDS support for RMW: ${rmw_implementation}")
  endif()
endmacro()
