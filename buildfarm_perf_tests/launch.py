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

from launch.actions import EmitEvent
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LocalSubstitution
from launch_ros.actions import Node


class SystemMetricCollector(Node):
    """Action that attaches the system_metric_collector to a process."""

    def __init__(
        self,
        target_action,
        log_file,
        timeout=None,
        **kwargs
    ) -> None:
        """
        Construct a SystemMetricCollector action.

        Parameters
        ----------
        target_action : ExecuteProcess
            ExecuteProcess (or Node) instance to collect metrics on
        log_file : str
            Path to where the collected metrics should be written to
        timeout : int
            Maximum time to run the metrics collector if the target process does
            not exit

        """
        # These Node/ExecuteProcess arguments are invalid in this context
        # because we implicitly set them right here.
        assert 'arguments' not in kwargs
        assert 'package' not in kwargs
        assert 'node_executable' not in kwargs
        assert 'executable' not in kwargs

        self.__pid_var_name = '__PROCESS_ID_%d' % id(self)

        kwargs['package'] = 'buildfarm_perf_tests'
        kwargs['node_executable'] = 'system_metric_collector'
        kwargs['arguments'] = [
            '--log', log_file,
            '--process_pid', LocalSubstitution(self.__pid_var_name)]
        if timeout is not None:
            kwargs['arguments'] += ['--timeout', str(timeout)]

        super().__init__(**kwargs)

        self.__target_start_handler = OnProcessStart(
            target_action=target_action, on_start=self.__on_target_start)
        self.__target_exit_handler = OnProcessExit(
            target_action=target_action, on_exit=EmitEvent(
                event=ShutdownProcess(
                    process_matcher=matches_action(self))))

    def execute(self, context):
        context.register_event_handler(self.__target_start_handler)

    def __on_target_start(self, event, context):
        context.extend_locals({self.__pid_var_name: str(event.pid)})
        context.register_event_handler(self.__target_exit_handler)
        super().execute(context)
