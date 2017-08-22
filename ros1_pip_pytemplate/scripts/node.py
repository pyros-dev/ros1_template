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
import requests  # needed to get status codes definition

import rospy
import ros1_template_msgs.msg as ros1_template_msgs
import ros1_template_msgs.srv as ros1_template_srvs

from ros1_pip_pytemplate import Httpbin

##############################################################################
# ROS
##############################################################################


# We define the exception here instead of inside the library,
# to keep side-effects outside of the core functionality
# Another client of the library might want to treat status code differently,
# but for ROS, treating non-OK status code as exception makes sense.
class StatusCodeException(Exception):
    pass

httpbin = Httpbin()


def callback(data):
    response = httpbin.get(params={a.key: a.value for a in data.args})
    if response.status_code == requests.status_codes.codes.OK:
        return ros1_template_srvs.GetResponse(
            origin=response.json().get('origin'),
            url=response.json().get('url'),
            args=[ros1_template_msgs.Arg(key=k, value=v)
                  for k, v in response.json().get('args').items()],
        )
    else:
        raise StatusCodeException(response.status_code)


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

    # Here we have parsed all arguments

    # We can now init the node (a ROS node is a process, that is an instance of the python interpreter)
    rospy.init_node('httpbin')

    # retrieving ros parameters
    base_url = rospy.get_param("~base_url")

    # setting up the proxy service
    rospy.Service('~get', ros1_template_srvs.Get, callback)

    # Just spin for ever, everything else is reactive !
    rospy.spin()

    rospy.logwarn("{0} is shutting down !".format(rospy.get_name()))
