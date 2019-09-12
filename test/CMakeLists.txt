find_package(launch_testing_ament_cmake REQUIRED)

function(add_performance_test COMM)
  set(MAX_RUNTIME 20)
  set(TOPIC "Array1k")
  configure_file(
    test_performance.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/test_performance_${COMM}.py.configure
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${COMM}.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_performance_${COMM}.py.configure"
  )
  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/test_performance_${COMM}.py"
    TARGET test_performance_${COMM}
  )
endfunction()

add_performance_test(ROS2)
