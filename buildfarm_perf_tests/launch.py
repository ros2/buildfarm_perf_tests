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

from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def attach_system_metric_collector(process, log_file, timeout=None):
    """Receives a Node Class to get the pid and add this id to system_metric_collector.
    
    Parameters
    ----------
    process : Node
        The Node to get the pid
    log_file : str
        path to the log file
    timeout: int
         time to keep alive system_metric_collector
         
    """
    pid_var_name = 'PROCESS_ID_%d' % id(process)

    args = [
        '--log', log_file,
        '--process_pid', EnvironmentVariable(pid_var_name)]

    if timeout is not None:
        args += ['--timeout', str(timeout)]

    system_metric_collector = Node(
        package='buildfarm_perf_tests',
        node_executable='system_metric_collector',
        arguments=args)

    def on_process_start(process_started, launch_context):
        SetEnvironmentVariable(
            pid_var_name, str(process_started.pid)).execute(launch_context)
        system_metric_collector.execute(launch_context)

    process_start_event = RegisterEventHandler(OnProcessStart(
        target_action=process, on_start=on_process_start))

    return system_metric_collector, process_start_event
