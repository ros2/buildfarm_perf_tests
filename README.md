# buildfarm perf tests

## Purpose

This package defines some tests which invoke `perf_test` from Apex.AI's [performance_test](https://gitlab.com/ApexAI/performance_test) package. This allows you to test performance and latency of several ROS 2 RMW implementations.

* There is a test for each RMW:

  - CycloneDDS (standalone test)
  - FastRTPS (standalone test)
  - rmw_connext_cpp
  - rmw_cyclonedds_cpp
  - rmw_fastrtps_cpp
  - rmw_fastrtps_dynamic_cpp
  - rmw_opensplice_cpp

##  Build

1.  Install ROS 2 (https://index.ros.org/doc/ros2/Installation/).
1.  Source the ROS 2 installation (either `/opt/ros/<rosdistro>/setup.bash` if installing from binaries, or `~/ros2_ws/install/setup.bash` if building from source):
    1.  `source /opt/ros/<rosdistro>/setup.bash` or `source ~/ros2_ws/install/setup.bash`
1.  Make a new workspace and clone this repository into it:
    1.  `mkdir -p ~/performance_ws/src`
    1.  `cd ~/performance_ws`
    1.  `wget https://github.com/ros2/buildfarm_perf_tests/raw/master/tools/ros2_dependencies.repos`
    1.  `vcs import src < ros2_dependencies.repos`
1.  Build the local workspace:
    1.  `colcon build`
1.  Source the local workspace:
    1.  `source install/local_setup.bash`

### Run

```bash
colcon test --packages-select buildfarm_perf_tests -DPERF_TEST_MAX_RUNTIME="30" -DPERF_TEST_TOPIC="Array1k" --event-handlers console_direct+
```

Add at the end the flags `--event-handlers console_direct+` if you want to visualize all the output.

## Details

* Each test runs for **30** seconds with a **1k payload**, but this can be changed using CMake variables.
 - `PERF_TEST_MAX_RUNTIME`: Maximum number of seconds to run before  exiting. Zero runs forever.
 - `PERF_TEST_TOPIC`: Topic to use. These are all available topics: `Array1k`, `Array4k`, `Array16k`, `Array32k`, `Array60k`, `Array1m`, `Array2m`, `Struct16`, `Struct256`, `Struct4k`, `Struct32k`, `PointCloud512k`, `PointCloud1m`, `PointCloud2m`, `PointCloud4m`, `Range`, `NavSatFix`, `RadarDetection` and `RadarTrack`.

For example, If we want to run the test during `30` seconds using the topic `Array1k`:

```bash
colcon build --packages-select buildfarm_perf_tests --cmake-args -DPERF_TEST_MAX_RUNTIME="30" -DPERF_TEST_TOPIC="Array1k"
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
  --process_name arg      process_name
  --process_arguments arg process_arguments
```

A general overview of what a typical run might do, for example:

1. Start process under test. For example `perf_test`
2. Launch `system_metrics_collector` using the argument  `--process_name` with the name of the process (in this case `perf_test`). You can also use the argument `--process_argument` to include one of the arguments of the `perf_test`, For example if you are running `perf_test` with `--roundtrip_mode` you can include in this option `Main` or `Relay` to identify each one of the processes.
3. Finally `system_metrics_collector` will fetch the data from the files describe above. If you have include the option `--log` then the data it's recorded in the file otherwise the standard output will show the reading.

### Examples

Example 1:

```bash
ros2 run performance_test perf_test -c FastRTPS -t Array1k
ros2 run buildfarm_perf_tests system_metric_collector --process_name perf_test
```

Example 2:

```bash
ros2 run performance_test perf_test -c FastRTPS -t Array1k --roundtrip_mode Relay
ros2 run performance_test perf_test -c FastRTPS -t Array1k --roundtrip_mode Main
ros2 run buildfarm_perf_tests system_metric_collector --process_name perf_test --process_argument Main
ros2 run buildfarm_perf_tests system_metric_collector --process_name perf_test --process_argument Relay
```
