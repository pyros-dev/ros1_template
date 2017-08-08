#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros1_pytemplate'],
    scripts=[
        'scripts/question_cli.py',
        'nodes/answer_server_node.py',
        'nodes/fibonacci_sub_node.py',
        'nodes/fibonacci_pub_node.py',
    ],
)

setup(**d)
