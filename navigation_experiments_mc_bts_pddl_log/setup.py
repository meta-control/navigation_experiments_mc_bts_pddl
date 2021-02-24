#!/usr/bin/env python

from setuptools import find_packages
from setuptools import setup

package_name = 'navigation_experiments_mc_bts_pddl_log'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jonatan Gines',
    author_email='jonatan.gines@urjc.es',
    maintainer='Jonatan Gines',
    maintainer_email='jonatan.gines@urjc.es',
    keywords=['ROS2', 'csv'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'The navigation_experiments_mc_bts_pddl_log package'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topics_2_csv = navigation_experiments_mc_bts_pddl_log.topics_2_csv:main',
            'reconfig_time = navigation_experiments_mc_bts_pddl_log.reconfig_time:main'
        ],
    },
)
