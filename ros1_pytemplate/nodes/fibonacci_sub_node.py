#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
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
import ros1_template_msgs.msg as ros1_template_msgs


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


def callback(data):
    rospy.loginfo("The next Fibonacci Number was heard: {0}".format(data.number))
    # return data is ignored


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
    rospy.init_node('fibonacci_sub_node')

    # setting up the subscriber to the topic
    fibonacci_pub = rospy.Subscriber('~fibonacci', ros1_template_msgs.Fibonacci, callback)

    # Just spin for ever, everything else is reactive !
    rospy.spin()

    rospy.logwarn("{0} is shutting down !".format(rospy.get_name()))
