# buildfarm perf tests

[![Continuous Integration](https://github.com/ros2/buildfarm_perf_tests/workflows/Continuous%20Integration/badge.svg?branch=master&event=push)](https://github.com/ros2/buildfarm_perf_tests/actions?query=branch%3Amaster+event%3Apush)

## Purpose

This package defines some tests. On one hand it invokes `perf_test` from Apex.AI's [performance_test](https://gitlab.com/ApexAI/performance_test) package. This allows you to test performance and latency of several ROS 2 RMW implementations. On the other hand we are evaluating the additional overhead caused by a single pub/sub topic or one process spinning and detect potential leaks related to theses activities.

* There is a test for each RMW:

  - CycloneDDS (standalone test)
  - FastRTPS (standalone test)
  - rmw_connext_cpp
  - rmw_cyclonedds_cpp
  - rmw_fastrtps_cpp
  - rmw_fastrtps_dynamic_cpp
  - rmw_opensplice_cpp

### Test 1 - Performance Test  (Apex.AI)

In this test we are running the Performance Test provided by Apex.AI. Right now we have [our own fork](https://github.com/ros2/performance_test) because there are some pending pull requests in the official gitlab repository.

In this test we are measurement:
 - Average latency
 - CPU usage (provided by Apex.AI tool)
 - Sent/Received messages per second
 - Total lost messages
 - Max resident set size

We are generating two plots per measurement
 - [One per-build](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/lastBuild/)
 - [Other over time](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/plot/)

**The test only measures the latency between the same RMW implementation**

| Publisher/Subscriber     | rmw_fastrtps_cpp         | rmw_opensplice_cpp       | rmw_cyclonedds_cpp       | rmw_fastrtps_dynamic_cpp | rmw_connext_cpp          |
|--------------------------|--------------------------|--------------------------|--------------------------|--------------------------|--------------------------|
| rmw_fastrtps_cpp         | :heavy_check_mark:       | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_multiplication_x: |
| rmw_opensplice_cpp       | :heavy_multiplication_x: | :heavy_check_mark:       | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_multiplication_x: |
| rmw_cyclonedds_cpp       | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_check_mark:       | :heavy_multiplication_x: | :heavy_multiplication_x: |
| rmw_fastrtps_dynamic_cpp | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_check_mark:       | :heavy_multiplication_x: |
| rmw_connext_cpp          | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_multiplication_x: | :heavy_check_mark:       |

## Test 2 - Simple pub/sub

In this case we are testing one publisher and one subscriber **in different processes** sending a 1kArray at 5Hz. This will allow us to evaluate additional overhead caused by a single pub/sub topic and detect leaks related to this activity.

We measure for both publisher and subscriber:

 - Average round trip
 - CPU usage ( readed from the filesystem )
 - Total lost messages
 - Received/Sent messages per second
 - Physical memory
 - Resident anonymous memory
 - Virtual memory

Again we plot measurement:
 - [One per-build](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/lastBuild/)
 - [Other over time](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/plot/)

| Publisher/Subscriber     | rmw_fastrtps_cpp   | rmw_opensplice_cpp | rmw_cyclonedds_cpp | rmw_fastrtps_dynamic_cpp | rmw_connext_cpp    |
|--------------------------|--------------------|--------------------|--------------------|--------------------------|--------------------|
| rmw_fastrtps_cpp         | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark:       | :heavy_check_mark: |
| rmw_opensplice_cpp       | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark:       | :heavy_check_mark: |
| rmw_cyclonedds_cpp       | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark:       | :heavy_check_mark: |
| rmw_fastrtps_dynamic_cpp | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark:       | :heavy_check_mark: |
| rmw_connext_cpp          | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark:       | :heavy_check_mark: |

## Test 3 - Node spinning

This test creates one process and spins for 1 minute to evaluate ROS 2 overhead and detect obvious leaks.

We measure:

 - CPU usage ( readed from file system )
 - Physical memory
 - Resident anonymous memory
 - Virtual memory

Again we plot measurement:
 - [One per-build](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/lastBuild/)
 - [Other over time](http://build.ros2.org/view/Rci/job/Rci__nightly-performance_ubuntu_focal_amd64/plot/Node%20Spinning%20Results/)

| DDS                      | Process 1 |
|--------------------------|-----------|
| rmw_fastrtps_cpp         | :heavy_check_mark:   |
| rmw_opensplice_cpp       |  :heavy_check_mark:   |
| rmw_cyclonedds_cpp       | :heavy_check_mark:   |
| rmw_fastrtps_dynamic_cpp |  :heavy_check_mark:   |
| rmw_connext_cpp          |   :heavy_check_mark:   |

##  Build

1.  Install ROS 2 (https://index.ros.org/doc/ros2/Installation/).
1.  Source the ROS 2 installation (either `/opt/ros/<rosdistro>/setup.bash` if installing from binaries, or `~/ros2_ws/install/setup.bash` if building from source):
    1.  `source /opt/ros/<rosdistro>/setup.bash` or `source ~/ros2_ws/install/setup.bash`
1.  Make a new workspace and clone this repository into it:
    1.  `mkdir -p ~/performance_ws/src`
    1.  `cd ~/performance_ws`
    1.  `wget https://github.com/ros2/buildfarm_perf_tests/raw/master/tools/ros2_dependencies.repos`
    1.  `vcs import src < ros2_dependencies.repos`
    1. `rosdep install --from-path src --ignore-src`
1.  Build the local workspace:
    1.  `colcon build`
1.  Source the local workspace:
    1.  `source install/local_setup.bash`

### Run

```bash
colcon test --packages-select buildfarm_perf_tests --event-handlers console_direct+
```

Add at the end the flags `--event-handlers console_direct+` if you want to visualize all the output.

## Details

  ***Note: the graphs presented here are for demonstration purposes only. The data in the graphs are not meant to be accurate or current.***

* Each test runs for **30** seconds with a **1k payload**, but this can be changed using CMake variables.
 - `PERF_TEST_RUNTIME`: Maximum number of seconds to run before  exiting. Zero runs forever.
 - `PERF_TEST_TOPICS`: Topic to use. These are all available topics: `Array1k`, `Array4k`, `Array16k`, `Array32k`, `Array60k`, `Array1m`, `Array2m`, `Struct16`, `Struct256`, `Struct4k`, `Struct32k`, `PointCloud512k`, `PointCloud1m`, `PointCloud2m`, `PointCloud4m`, `Range`, `NavSatFix`, `RadarDetection` and `RadarTrack`.

For example, If we want to run the test during `30` seconds using the topic `Array1k`:

```bash
colcon build --packages-select buildfarm_perf_tests --cmake-args -DPERF_TEST_RUNTIME="30" -DPERF_TEST_TOPICS="Array1k"
```

* Each test produces a PNG plot of [various measures](http://build.ros2.org/view/Eci/job/Eci__nightly-performance_ubuntu_bionic_amd64/) across time, displayed in Jenkins using the image gallery plugin.
  - These plots are displayed on the build's summary page, and are part of the output artifacts for the build.
![](img/latency.png)

* Each test also produces a couple of [aggregated measures](http://build.ros2.org/view/Eci/job/Eci__nightly-performance_ubuntu_bionic_amd64/plot/) in a small csv file, used to plot build-over-build using the Jenkins plot plugin.

   - To see these plots, click the "Plots" link on the left side of the JOB summary (not a build summary)
   - You should be able to click one of those points to jump to the aforementioned PNG plot that produced that aggregated point.
![](img/agregate_latency.png)
![](img/size.png)
![](img/cpu.png)
![](img/lost_packets.png)
![](img/received_packets.png)
![](img/sent_packets.png)

## System metrics collector tool

This tool allows to create statistics based on the name of the process and arguments. This tool allows to collect the following statistics:

 - CPU usage (%): This information is fetched from `/proc/stat`.
 - CPU memory virtual, ResidentAnonymousMemory and physical: This information is fetched from `/proc/meminfo`.
 - Process usage (%):  This information is fetched from `/proc/<pid/>stat`.
 - Process memory: a) virtual, b) resident anonymous memory and c) physical:  This information is fetched from  `/proc/<pid/statm` and `/proc/<pid/status`.

These are the argument to launch the tool:

```
ros2 run buildfarm_perf_tests system_metric_collector -h
Options:
  -h [ --help ]           Help screen
  --timeout arg (=60)     Test duration
  --log arg (=out.csv)    Log filename
  --process_pid arg      process pid
```

A general overview of what a typical run might do, for example:

1. Start process under test. For example `perf_test`
2. Launch `system_metrics_collector` using the argument `--process_pid` with the pid of the process (in this case `perf_test`).
3. Finally `system_metrics_collector` will fetch the data from the files describe above. If you have include the option `--log` then the data it's recorded in the file otherwise the standard output will show the reading.

### Examples

```bash
ros2 run performance_test perf_test -c ROS2 -t Array1k &
ps -e | grep perf_test
  8621 pts/5    00:00:01 perf_test
ros2 run buildfarm_perf_tests system_metric_collector -process_pid 8621
```
