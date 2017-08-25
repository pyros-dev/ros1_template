#!/usr/bin/env python

# Following http://packaging.python.org
# This will be used by catkin_pip when building the ROS package.
from setuptools import setup

# Ref : https://packaging.python.org/single_source_version/#single-sourcing-the-version
with open('ros1_pip_pytemplate/_version.py') as vf:
    exec(vf.read())

# IMPORTANT : to work with this package (as with any python package),
# you should always work in a virtual environment,
# to avoid polluting your system with random packages of random versions.
# CHECK : https://virtualenvwrapper.readthedocs.io/en/latest
setup(
    name='ros1_pip_pytemplate',
    version=__version__,
    packages=[
        'ros1_pip_pytemplate',
        'ros1_pip_pytemplate.tests',
    ],
    install_requires=[
        'pyros-setup',  # pyros-setup is needed, but only when running from python environment

        # a pure pip package : this package will not work properly from ROS install space or deb package,
        # unless requests is also available from ROS, and declared in package.xml for rosdep to resolve it.
        # Look for it in https://github.com/ros/rosdistro/blob/master/python.yaml
        # From source, we rely on basic python/pip system for it however (and we don't rely on ros dependency system).
        'requests',
        # same for pyyaml
        'pyyaml',
        # only for embedded tests
        'mock'
    ],
    tests_require=[
        'pytest',
    ],
    scripts=[
        'scripts/cli.py',
        'scripts/node_reactive.py',
        #'scripts/node_proactive.py',
    ],
    author="AlexV",
    author_email="asmodehn@gmail.com",
    description="Template ROS node for use with catkin_pip following good practices",
    license="MIT",
)
