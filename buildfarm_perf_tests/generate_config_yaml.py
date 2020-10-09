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

import os
from string import Template

from ament_index_python import get_package_share_directory


def _add_line_plot_overhead(file_output, template_name, label, rmw_implementations, ci_name):
    """Add line to plot overhead."""
    with open(template_name, 'r') as content_file:
        content = Template(content_file.read())

    file_output.write('  ' + label + '\n')

    name = os.path.splitext(os.path.basename(template_name))[0]
    for rmw_implementation in rmw_implementations:
        random_number = _check_unique_identifier(name + '_' + rmw_implementation)
        file_output.write(content.substitute(rmw_implementation=rmw_implementation,
                                             ci_name=ci_name,
                                             random_number=random_number))


_unique_identifiers = set()


def _check_unique_identifier(name):
    global _unique_identifiers
    unique_identifier = name
    assert unique_identifier not in _unique_identifiers, unique_identifier
    _unique_identifiers.add(unique_identifier)
    return unique_identifier


def _fill_performance_test(file_output, template_name, label, ci_name):
    """Add line to performance test."""
    with open(template_name, 'r') as content_file:
        content = Template(content_file.read())

    file_output.write('  ' + label + '\n')

    name = os.path.splitext(os.path.basename(template_name))[0]
    random_number = _check_unique_identifier(name)
    file_output.write(content.substitute(ci_name=ci_name,
                                         random_number=random_number))


def performance_test(file_output, ci_name):
    """Generate yaml file for performance tests."""
    templates_path = os.path.join(get_package_share_directory('buildfarm_perf_tests'), 'templates')

    _fill_performance_test(file_output,
                           os.path.join(templates_path, 'performance_test_1p_1k.txt'),
                           'Performance One Process Test Results (Array1k):',
                           ci_name)
    _fill_performance_test(file_output,
                           os.path.join(templates_path, 'performance_test_1p_multi.txt'),
                           'Performance One Process Test Results (multisize messages):',
                           ci_name)
    _fill_performance_test(file_output,
                           os.path.join(templates_path, 'performance_test_2p_1k.txt'),
                           'Performance Two Processes Test Results (Array1k):',
                           ci_name)
    _fill_performance_test(file_output,
                           os.path.join(templates_path, 'performance_test_2p_multi.txt'),
                           'Performance Two Processes Test Results (multisize messages):',
                           ci_name)


def node_spinnnig(file_output, ci_name):
    """Generate yaml file for node spinning test."""
    templates_path = os.path.join(get_package_share_directory('buildfarm_perf_tests'), 'templates')
    with open(os.path.join(templates_path, 'node_spinning.txt'), 'r') as content_file:
        content = Template(content_file.read())

    file_output.write('  Node Spinning Results:\n')

    random_number = _check_unique_identifier('node_spinning')
    file_output.write(content.substitute(ci_name=ci_name,
                                         random_number=random_number))


def overhead_simple_publisher_and_subscriber(file_output, rmw_implementations, ci_name):
    """Generate yaml file for overhead simple publisher and subscriber tests."""
    templates_path = os.path.join(get_package_share_directory('buildfarm_perf_tests'), 'templates')
    _add_line_plot_overhead(file_output,
                            os.path.join(templates_path, 'overhead_round_trip.txt'),
                            'Overhead simple publisher and subscriber - Average Round-Trip Time:',
                            rmw_implementations, ci_name)
    _add_line_plot_overhead(
        file_output,
        os.path.join(templates_path, 'overhead_received_messages.txt'),
        'Overhead simple publisher and subscriber - Received messages per second:',
        rmw_implementations,
        ci_name)
    _add_line_plot_overhead(file_output,
                            os.path.join(templates_path, 'overhead_sent_messages.txt'),
                            'Overhead simple publisher and subscriber - Sent messages per second:',
                            rmw_implementations, ci_name)
    _add_line_plot_overhead(file_output,
                            os.path.join(templates_path, 'overhead_lost_messages.txt'),
                            'Overhead simple publisher and subscriber - Lost messages per second:',
                            rmw_implementations, ci_name)
    _add_line_plot_overhead(file_output,
                            os.path.join(templates_path, 'overhead_virtual_memory.txt'),
                            'Overhead simple publisher and subscriber - Virtual Memory:',
                            rmw_implementations, ci_name)
    _add_line_plot_overhead(
        file_output,
        os.path.join(templates_path, 'overhead_resident_anonymous_memory.txt'),
        'Overhead simple publisher and subscriber - Resident Anonymous Memory:',
        rmw_implementations,
        ci_name)
    _add_line_plot_overhead(file_output,
                            os.path.join(templates_path, 'overhead_physical_memory.txt'),
                            'Overhead simple publisher and subscriber - Physical Memory:',
                            rmw_implementations, ci_name)
