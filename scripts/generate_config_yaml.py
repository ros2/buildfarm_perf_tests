#!/usr/bin/python3

import argparse
import os
import random
from string import Template

from ament_index_python import get_package_share_directory


def add_line_to_plot_overhead(file_output, template_name, label, rmw_implementations, ci_name):
    """Add line to plot overhead."""
    with open(template_name, 'r') as content_file:
        content = Template(content_file.read())

    file_output.write('  ' + label + '\n')

    for rmw_implementation in rmw_implementations:
        random_number = rand_x_digit_num(19)
        file_output.write(content.substitute(rmw_implementation=rmw_implementation,
                                             ci_name=ci_name,
                                             random_number=random_number))


def rand_x_digit_num(x):
    """Return an X digit number, leading_zeroes returns a string, otherwise int."""
    return '{0:0{x}d}'.format(random.randint(0, 10**x-1), x=x)


def fill_performance_test(file_output, template_name, label, ci_name):
    """Add line to performance test."""
    with open(template_name, 'r') as content_file:
        content = Template(content_file.read())

    file_output.write('  ' + label + '\n')

    random_number = rand_x_digit_num(19)
    file_output.write(content.substitute(ci_name=ci_name,
                                         random_number=random_number))


def performance_test(file_output, ci_name):
    """Generate yaml file for performance tests."""
    templates_path = os.path.join(get_package_share_directory('buildfarm_perf_tests'), 'templates')

    fill_performance_test(file_output,
                          os.path.join(templates_path, 'performance_test_1p_1k.txt'),
                          'Performance One Process Test Results (Array1k)',
                          ci_name)
    fill_performance_test(file_output,
                          os.path.join(templates_path, 'performance_test_1p_multi.txt'),
                          'Performance One Process Test Results (multisize packets):',
                          ci_name)
    fill_performance_test(file_output,
                          os.path.join(templates_path, 'performance_test_2p_1k.txt'),
                          'Performance Two Processes Test Results (Array1k)',
                          ci_name)
    fill_performance_test(file_output,
                          os.path.join(templates_path, 'performance_test_2p_multi.txt'),
                          'Performance Two Processes Test Results (multisize packets):',
                          ci_name)


def node_spinnnig(file_output, ci_name):
    """Generate yaml file for node spinning test."""
    templates_path = os.path.join(get_package_share_directory('buildfarm_perf_tests'), 'templates')
    with open(os.path.join(templates_path, 'node_spinning.txt'), 'r') as content_file:
        content = Template(content_file.read())

    file_output.write('  Node Spinnig Results:\n')

    random_number = rand_x_digit_num(19)
    file_output.write(content.substitute(ci_name=ci_name,
                                         random_number=random_number))


def overhead_simple_publisher_and_subscriber(file_output, rmw_implementations, ci_name):
    """Generate yaml file for overhead simple publisher and subscriber tests."""
    templates_path = os.path.join(get_package_share_directory('buildfarm_perf_tests'), 'templates')
    add_line_to_plot_overhead(file_output,
                              os.path.join(templates_path, 'overhead_round_trip.txt'),
                              'Overhead simple publisher and subscriber - Average Round-Trip Time:',
                              rmw_implementations, ci_name)
    add_line_to_plot_overhead(
                file_output,
                os.path.join(templates_path, 'overhead_received_packets.txt'),
                'Overhead simple publisher and subscriber - Received packets per second:',
                rmw_implementations,
                ci_name)
    add_line_to_plot_overhead(file_output,
                              os.path.join(templates_path, 'overhead_sent_packets.txt'),
                              'Overhead simple publisher and subscriber - Sent packets per second:',
                              rmw_implementations, ci_name)
    add_line_to_plot_overhead(file_output,
                              os.path.join(templates_path, 'overhead_lost_packets.txt'),
                              'Overhead simple publisher and subscriber - Lost packets per second:',
                              rmw_implementations, ci_name)
    add_line_to_plot_overhead(file_output,
                              os.path.join(templates_path, 'overhead_virtual_memory.txt'),
                              'Overhead simple publisher and subscriber - Virtual Memory:',
                              rmw_implementations, ci_name)
    add_line_to_plot_overhead(
                file_output,
                os.path.join(templates_path, 'overhead_resident_anonymous_memory.txt'),
                'Overhead simple publisher and subscriber - Resident Anonymous Memory:',
                rmw_implementations,
                ci_name)
    add_line_to_plot_overhead(file_output,
                              os.path.join(templates_path, 'overhead_physical_memory.txt'),
                              'Overhead simple publisher and subscriber - Physical Memory:',
                              rmw_implementations, ci_name)


def main():
    """Call main function."""
    parser = argparse.ArgumentParser(description='Create ros buildfarm config.')
    parser.add_argument('-filename', dest='filename', required=True,
                        help='output file', metavar='FILE')
    parser.add_argument('-ci_name', dest='ci_name', required=True,
                        help='ci_name job', metavar='ci_name',)
    parser.add_argument('-rmw_implementations', metavar='rmw_implementations', nargs='+',
                        help='This will integrate the rmw_implementation in the plots')
    args = parser.parse_args()

    with open(args.filename, 'w') as f:
        overhead_simple_publisher_and_subscriber(f, args.rmw_implementations, args.ci_name)
        node_spinnnig(f, args.ci_name)
        performance_test(f, args.ci_name)


main()
