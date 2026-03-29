# rclpy Spinning Overhead Benchmark

This document covers the Python spinning-node overhead benchmark added as a sibling to the existing C++ `node_spinning` benchmark.

## What Was Added

- `buildfarm_perf_tests/node_spinning_rclpy.py`
  - Minimal `rclpy` node that initializes ROS 2 and spins until shutdown.
- `scripts/node_spinning_rclpy`
  - Installed executable used by `ros2 run buildfarm_perf_tests node_spinning_rclpy`.
- `test/add_performance_tests.cmake`
  - Generates `test_spinning_rclpy_<rmw>_<sync>.py` launch tests next to the existing C++ spinning tests.
- `test/test_spinning.py.in`
  - Reused for both implementations by parameterizing the executable name and result metadata.

## How It Works

The benchmark reuses the same launch-testing architecture as the C++ spinning benchmark:

1. A generated launch test starts the target executable.
2. `SystemMetricCollector` waits for the process start event and attaches to its PID.
3. The collector samples `/proc`-based CPU and memory metrics for the benchmark duration.
4. After shutdown, the test converts the raw CSV into Jenkins-style CSV, JSON, and PNG outputs.

The Python benchmark is intentionally minimal. It measures the overhead of a process that creates an `rclpy` node and stays inside `rclpy.spin(node)` until the launch test stops it.

## Files and Responsibilities

- `buildfarm_perf_tests/node_spinning_rclpy.py`
  - Benchmark implementation.
- `scripts/node_spinning_rclpy`
  - Installed ROS 2 executable wrapper.
- `test/test_spinning.py.in`
  - Shared launch-testing template for both C++ and Python spinning benchmarks.
- `test/add_performance_tests.cmake`
  - Generates per-RMW, per-sync-mode launch tests for the Python variant.

## Manual Run

After building and sourcing the workspace:

```bash
ros2 run buildfarm_perf_tests node_spinning_rclpy
```

To attach the existing metric collector manually:

```bash
ros2 run buildfarm_perf_tests node_spinning_rclpy &
SPINNING_PID=$!
ros2 run buildfarm_perf_tests system_metric_collector \
  --process_pid "$SPINNING_PID" \
  --timeout 20 \
  --log /tmp/node_spinning_rclpy_metrics.csv
kill "$SPINNING_PID"
```

Inspect the raw collector output with:

```bash
head /tmp/node_spinning_rclpy_metrics.csv
```

## Run Through the Test Harness

The generated tests follow this naming scheme:

```text
test_spinning_rclpy_<rmw_implementation>_<sync_mode>
```

Example:

```bash
colcon test \
  --packages-select buildfarm_perf_tests \
  --ctest-args -R test_spinning_rclpy_rmw_fastrtps_cpp_async --output-on-failure
```

## Developer Guide: Run Only This Benchmark

Build just this package:

```bash
colcon build --packages-select buildfarm_perf_tests
source install/local_setup.bash
```

List the generated Python spinning tests:

```bash
cd buildfarm_perf_tests
ctest -N | grep test_spinning_rclpy
```

Run one benchmark variant only:

```bash
colcon test \
  --packages-select buildfarm_perf_tests \
  --ctest-args -R test_spinning_rclpy --output-on-failure
```

Narrow further to a specific RMW or sync mode with a more specific regex.

## Output Artifacts

When `PERF_TEST_RESULTS_BASE_PATH` is set by the generated launch test, the benchmark writes:

- `<base>.csv`
  - Aggregated Jenkins plot values.
- `<base>.benchmark.json`
  - Jenkins benchmark plugin data, including `client_library=rclpy`.
- `<base>_virtual_memory.png`
- `<base>_cpu_usage.png`
- `<base>_physical_memory.png`
- `<base>_resident_anonymous_memory.png`

The raw metric collector CSV is created in a temporary file during the launch test, then converted into the artifacts above.

## Metric Collection Details

`SystemMetricCollector` is unchanged. It still launches `system_metric_collector` from this package and passes the benchmark PID via launch events. The collector records:

- process CPU usage
- virtual memory
- physical memory
- resident anonymous memory

This keeps the new Python benchmark inside the current result flow and avoids introducing a second measurement stack.