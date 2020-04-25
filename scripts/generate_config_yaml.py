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

import argparse

from buildfarm_perf_tests.generate_config_yaml import node_spinnnig
from buildfarm_perf_tests.generate_config_yaml import overhead_simple_publisher_and_subscriber
from buildfarm_perf_tests.generate_config_yaml import performance_test


def main():
    """Call main function."""
    parser = argparse.ArgumentParser(description='Create ros buildfarm config.')
    parser.add_argument('-f', '--filename', dest='filename', required=True,
                        help='output file', metavar='FILE')
    parser.add_argument('-c', '--ci_name', dest='ci_name', required=True,
                        help='ci_name job', metavar='ci_name',)
    parser.add_argument('-rmw', '--rmw_implementations', metavar='rmw_implementations', nargs='+',
                        help='This will integrate the rmw_implementation in the plots')
    args = parser.parse_args()

    with open(args.filename, 'w') as f:
        overhead_simple_publisher_and_subscriber(f, args.rmw_implementations, args.ci_name)
        node_spinnnig(f, args.ci_name)
        performance_test(f, args.ci_name)


main()
