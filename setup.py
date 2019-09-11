#!/usr/bin/env python3
import sys

from setuptools import setup


if sys.version_info < (3, 5):
    print('ament requires Python 3.5 or higher.', file=sys.stderr)
    sys.exit(1)

package_name = 'buildfarm_perf_tests'
setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Scott K Logan',
    author_email='logans@cottsay.net',
    maintainer='Scott K Logan',
    maintainer_email='logans@cottsay.net',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Performance tests which run regularly on the buildfarm',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
