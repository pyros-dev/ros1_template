#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: Unspecified
#

from __future__ import absolute_import, division, print_function


"""
Module gathering all ROS side-effects
This module can however be run directly from a python virtual env
where pyros_setup has been installed and configured
pyros_setup will take care of setting up this process environment for ROS.
"""


##############################################################################
# Imports
##############################################################################
import argparse
import logging
import os
import sys
import requests  # needed to get status codes definition


import logging.config
logging.config.dictConfig(
    {
        'version': 1,
        'formatters': {
            'verbose': {
                'format': '%(levelname)s %(asctime)s %(module)s %(process)d %(thread)d %(message)s'
            },
            'simple': {
                'format': '%(levelname)s %(name)s:%(message)s'
            },
        },
        'handlers': {
            'console': {
                'level': 'INFO',
                'class': 'logging.StreamHandler',
                'formatter': 'simple',
                'stream': 'ext://sys.stdout',
            },
            'logfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'filename': 'ros1_pip_pytemplate.log',
                'maxBytes': 1024,
                'backupCount': 3,
                'formatter': 'verbose'
            },
        },
        'loggers': {
            'ros1_pip_pytemplate': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': True,
            },
        }
    }
)

##############################################################################
# ROS isolated in one function
# ROS initialization, if not done in the launch environment, :
# - doesn't corrupt the Python interpreter at (critical) import time,
# - is delayed until we actually need it.
# - can be called in a subprocess to never corrupt the initial environment.
# This is useful for tools, tests, etc. to operate in a usual python setup.
##############################################################################

def node_spin(name, argv=None):

    try:
        import rospy
        # This might break if ROS environment is not setup
    except ImportError:
        import pyros_setup
        pyros_setup.configurable_import().configure().activate()
        import rospy

    import ros1_pip_pytemplate

    # We define the exception here instead of inside the library,
    # to keep side-effects outside of the core functionality
    # Another client of the library might want to treat status code differently,
    # but for ROS, treating non-OK status code as exception makes sense.
    class StatusCodeException(Exception):
        pass

    # Note : log_level here is about rosgraph logs, not python logging logs
    rospy.init_node(name, argv=None)

    # retrieving ros parameters
    base_url = rospy.get_param("~base_url")

    httpbin = ros1_pip_pytemplate.Httpbin(base_url=base_url)

    # ROS environment is setup here
    import ros1_template_msgs.msg as ros1_template_msgs
    import ros1_template_msgs.srv as ros1_template_srvs

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

    # setting up the proxy service
    rospy.Service('~get', ros1_template_srvs.Get, callback)

    # Just spin for ever, everything else is reactive !
    rospy.spin()

    rospy.logwarn("{0} is shutting down !".format(rospy.get_name()))


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
    # Ref : https://docs.python.org/2/library/argparse
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(sys.argv[0]),
                                     epilog=show_epilog(),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    known_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    # Here we have parsed all arguments

    parser.add_argument("--version", action='store_true', help="display the version number and exits.")
    parser.add_argument(
        "--base_url",
        action='store',
        help="the base url to connect to. If not provided, it will be retrieved from the ~base_url ROS parameter.")

    parsed_known_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    print("known args: {0}".format(parsed_known_args))

    if parsed_known_args.version:
        print("ROS1 pip pytemplate version " + ros1_pip_pytemplate.__version__ +
              "\n from " + ros1_pip_pytemplate.__file__)
        sys.exit(0)

    if parsed_known_args.base_url:
        # Here we change the standard arguments syntax to match ROS syntax, and we'll let rospy.init_node() do its magic
        sys.argv.remove('--base_url'), sys.argv.remove(parsed_known_args.base_url)
        sys.argv.append('_base_url' + ':=' + parsed_known_args.base_url)

    # We can now init the node (a ROS node is a process - this process -, that is an instance of the python interpreter)
    node_spin('httpbin')

