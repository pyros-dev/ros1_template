#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: Unspecified
#

from __future__ import absolute_import, division, print_function

"""
Module gathering all ROS side-effects
"""


##############################################################################
# Imports
##############################################################################
import argparse
import os
import sys
import time

import rospy
import ros1_pytemplate
import ros1_template_msgs.msg as ros1_template_msgs

from dynamic_reconfigure.server import Server
from ros1_template_msgs.cfg import FibonacciConfig


##############################################################################
# Main
##############################################################################

def show_description():
    return "Fibonacci Publisher node"


def show_usage(cmd=None):
    cmd = os.path.relpath(sys.argv[0], os.getcwd()) if cmd is None else cmd
    return "{0} [-h|--help] [--version]".format(cmd)


def show_epilog():
    return "never enough testing"


if __name__ == '__main__':

    # we trim the arguments we received from the launcher
    args = rospy.myargv(sys.argv)

    # Ref : https://docs.python.org/2/library/argparse
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(),
                                     epilog=show_epilog(),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--version", action='store_true', help="display the version number and exits.")

    parsed_known_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    if parsed_known_args.version:
        print("ROS1 pytemplate version " + ros1_pytemplate.__version__ +
              "\n from " + ros1_pytemplate.__file__)
        sys.exit(0)

    # We can now init the node (a ROS node is a process, that is an instance of the python interpreter)
    rospy.init_node('fibonacci_pub_node')

    # retrieving ros parameters
    fib_init = rospy.get_param("~fib_init")
    fib_max = rospy.get_param("~fibonacci_max_number")

    # setting up the publisher to the topic
    fibonacci_pub = rospy.Publisher('~fibonacci', ros1_template_msgs.Fibonacci, queue_size=1)

    fib = ros1_pytemplate.Fibonacci(fib_init[0], fib_init[1], max=fib_max)

    rate = rospy.Rate(1)  # 1Hz

    # Setting up dynamic reconfigure server
    def dynamic_reconfigure_cb(config, level):
        rospy.loginfo("""Reconfigure Fibonacci Max request: {fibonacci_max_number}""".format(**config))
        # modify the fib object
        fib.max = config.fibonacci_max_number
        return config

    srv = Server(FibonacciConfig, dynamic_reconfigure_cb)

    try:
        # Just spin and publish for ever, be proactive !
        while not rospy.is_shutdown():
            # building the message we need to broadcast dynamically
            fib_number = ros1_template_msgs.Fibonacci(
                number=fib.next()
            )
            # logging it first
            rospy.loginfo("broadcasting : {0}".format(fib_number))
            # publishing it
            fibonacci_pub.publish(fib_number)
            # sleeping a bit to not burn the CPU
            rate.sleep()

    except Exception as exc:
        rospy.logwarn("{0} detected in node {1} : {2}".format(type(exc), rospy.get_name(), str(exc)))
    finally:
        rospy.logwarn("{0} is shutting down !".format(rospy.get_name()))
