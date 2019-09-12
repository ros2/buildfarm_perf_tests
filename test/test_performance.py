# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os
import unittest

from launch import LaunchDescription, LaunchService
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch_ros.actions import Node
from launch_testing import ActiveIoHandler

import matplotlib  # noqa: F401
import matplotlib.pyplot as plt
import numpy as np  # noqa: F401
import pandas as pd

plt.switch_backend('agg')


def _create_node(comm='ROS2', topic='Array1k', max_runtime=10, log_file=None):
    return Node(
        package='performance_test', node_executable='perf_test', output='log',
        arguments=[
            '-c', comm, '-t', topic, '--max_runtime', str(max_runtime)
        ] + (['-l', log_file] if log_file else []),
    )


def _csv_to_png(csv_path):
    dataframe = pd.read_csv(csv_path, sep='[ \t]*,[ \t]*', engine='python')
    pd.options.display.float_format = '{:.4f}'.format
    dataframe['maxrss (Mb)'] = dataframe['ru_maxrss'] / 1e3
    dataframe.drop(list(dataframe.filter(regex='ru_')), axis=1, inplace=True)
    dataframe['latency_variance (ms) * 100'] = 100.0 * dataframe['latency_variance (ms)']
    dataframe[['T_experiment',
               'latency_min (ms)',
               'latency_max (ms)',
               'latency_mean (ms)',
               'latency_variance (ms) * 100',
               'maxrss (Mb)']].plot(x='T_experiment', secondary_y=['maxrss (Mb)'])

    png_path = os.path.splitext(csv_path)[0] + '.png'
    plt.savefig(png_path)


class TestRMWPerformance(unittest.TestCase):

    def test_performance(self):
        proc_output = ActiveIoHandler()

        node = _create_node()
        ld = LaunchDescription([
            node,
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=proc_output.append,
                ),
            ),
        ])
        ls = LaunchService()
        ls.include_launch_description(ld)
        assert 0 == ls.run()

        text_lines = [l for t in proc_output for l in t.text.decode().splitlines()]
        line_start = text_lines.index('---EXPERIMENT-START---')
        line_end = text_lines.index('Maximum runtime reached. Exiting.')
        if line_end < 0:
            line_end = len(text_lines)
        assert line_start >= 0

        print('\n'.join(text_lines[:line_start]))

        csv_path = os.path.join(os.environ['PWD'], 'performance_test_results.csv')

        with open(csv_path, 'w') as f:
            for line in text_lines[line_start + 1:line_end]:
                f.write(line)
                f.write('\n')

        _csv_to_png(csv_path)
