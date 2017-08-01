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
import ros1_template
import ros1_template.msg as ros1_template_msgs


##############################################################################
# Main
##############################################################################

def show_description():
    return "ros template client test script"


def show_usage(cmd):
    cmd = cmd or sys.argv[0]
    return "{0} <origin_url> <replica_url> [--async]".format(cmd)


def show_epilog():
    return "never enough testing"


if __name__ == '__main__':

    # we trim the arguments we received from the launcher
    args = rospy.myargv(sys.argv)

    # Ref : https://docs.python.org/2/library/argparse
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(args[0]),
                                     epilog=show_epilog(),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parsed_args = parser.parse_args(args[1:])

    # Here we have parsed all CLI arguments

    # We can now init the node (a ROS node is a process, that is an instance of the python interpreter)
    rospy.init_node('prophet_node', )

    # retrieving ros parameters
    fib_init = rospy.get_param("~fib_init")

    # setting up the publisher to the topic
    fibonacci_pub = rospy.Publisher('~fibonacci', ros1_template_msgs.Fibonacci, queue_size=1)

    fib = ros1_template.Fibonacci(fib_init[0], fib_init[1])

    rate = rospy.Rate(1)  # 1Hz

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

