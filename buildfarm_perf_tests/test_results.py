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

from pandas import read_csv


def read_performance_test_csv(csv_path, start_marker='---EXPERIMENT-START---\n'):
    with open(csv_path, 'r') as csv:
        if start_marker:
            while csv.readline() not in [start_marker, '']:
                pass
        return read_csv(csv, sep='[ \t]*,[ \t]*', engine='python')


def write_jenkins_plot_csv(csv_path, column_name, values):
    with open(csv_path, 'w') as csv:
        csv.write(','.join([column_name] * len(values)) + '\n')
        csv.write(','.join(values) + '\n')
