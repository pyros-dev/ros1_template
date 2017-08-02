#!/usr/bin/env python

# Following http://packaging.python.org
# This will be used by catkin_pip when building the ROS package.
from setuptools import setup

# Ref : https://packaging.python.org/single_source_version/#single-sourcing-the-version
with open('ros1_pip_template/_version.py') as vf:
    exec(vf.read())

# IMPORTANT : to work with this package (as with any python package),
# you should always work in a virtual environment,
# to avoid polluting your system with random packages of random versions.
# CHECK : https://virtualenvwrapper.readthedocs.io/en/latest
setup(
    name='ros1_pip_template',
    version=__version__,
    packages=[
        'ros1_pip_template',
        'ros1_pip_template.tests',
    ],
    install_requires=[
        'pyros-setup',  # pyros-setup is needed, but only when running from python environment

        # a pure pip package : this package will not work properly from ROS install space or deb package,
        # unless requests is also available from ROS, and declared in package.xml for rosdep to resolve it.
        # Look for it in https://github.com/ros/rosdistro/blob/master/python.yaml
        # From source, we rely on basic python/pip system for it however (and we don't rely on ros dependency system).
        'requests',
    ],
    tests_require=[
        'pytest'
    ],
    scripts=[
        'scripts/httpbin_cli.py',
        'scripts/httpbin_node.py',
        'scripts/httpbin_launch.py',
    ],
    author="AlexV",
    author_email="asmodehn@gmail.com",
    description="Template ROS node for use with catkin_pip following good practices",
    license="MIT",
)