#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: Unspecified
#

from __future__ import absolute_import, division, print_function

import functools

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
from urlparse import urlsplit

import rospy
import ros1_template
import ros1_template.msg as ros1_template_msgs
import ros1_template.srv as ros1_template_srvs


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


def callback(answer, data):
    rospy.loginfo("The question was asked: {0}".format(data.question))
    return ros1_template_srvs.AnswerResponse(
        # filling up the fields in the response message one by one
        answer=answer.retrieve()
    )


def error_callback(data):
    rospy.logerr("An error will be triggered...")
    raise RuntimeError("ERROR ! No panic, it's just for testing.")
    # Notice how the node keeps running, and you can call this service again...


if __name__ == '__main__':

    # we trim the arguments we received from the launcher
    args = rospy.myargv(sys.argv)

    # Ref : https://docs.python.org/2/library/argparse
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(args[0]),
                                     epilog=show_epilog(),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parsed_args = parser.parse_args(args[1:])

    # Here we have parsed all arguments

    # We can now init the node (a ROS node is a process, that is an instance of the python interpreter)
    rospy.init_node('oracle_node', )

    # retrieving ros parameters
    answer_part = rospy.get_param("~answer_part")

    # Answer instance from ros_params:
    answer = ros1_template.Answer(answer_part)

    # setting up the service
    rospy.Service('~answer', ros1_template_srvs.Answer,
                  # we use a partial function application to pass initial Answer instance
                  functools.partial(callback, answer)
    )

    # setting up a service responding with an error
    rospy.Service('~error', ros1_template_srvs.Answer, error_callback)

    # Just spin for ever, everything else is reactive !
    rospy.spin()
