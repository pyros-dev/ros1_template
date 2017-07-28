#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros1_template'],
    scripts=[
        'scripts/question.py',
        'nodes/template_node.py',
    ],
)

setup(**d)
