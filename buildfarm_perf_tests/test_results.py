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

import json
from pandas import read_csv


def read_performance_test_csv(csv_path, start_marker='---EXPERIMENT-START---\n'):
    """
    Read CSV data from a file on disk, optionall skipping header data.

    Parameters
    ----------
    csv_path : str
        Path to the *.csv file to read data from.
    start_marker : str, optional
        Full line contents which mark the end of the header.

    """
    with open(csv_path, 'r') as csv:
        if start_marker:
            while csv.readline() not in [start_marker, '']:
                pass
        return read_csv(csv, sep='[ \t]*,[ \t]*', engine='python')


def write_jenkins_plot_csv(csv_path, column_name, values):
    """
    Write some values to a CSV file for the Jenkins plot plugin to use.

    Parameters
    ----------
    csv_path : str
        Path to where the *.csv file should be written to.
    column_name : str
        Name of this data series, such as the RMW or COMM value.
    values : list
        List of string to be written to the file.

    """
    with open(csv_path, 'w') as csv:
        csv.write(','.join([column_name] * len(values)) + '\n')
        csv.write(','.join([str(v) for v in values]) + '\n')


def write_jenkins_benchmark_json(json_path, sub_group, values):
    """
    Write some values to a JSON file for the Jenkins benchmark plugin to use.

    Parameters
    ----------
    json_path : str
        Path to where the *.json file should be written to.
    sub_group : str
        Optional supplementary identifier for the test group name.
    values : dict
        Mapping from measurement name to benchmark result to be written to the file.

    """
    group_name = 'buildfarm_perf_tests.' + sub_group
    out_data = {
        group_name: values,
    }
    with open(json_path, 'w') as json_file:
        json.dump(out_data, json_file)
