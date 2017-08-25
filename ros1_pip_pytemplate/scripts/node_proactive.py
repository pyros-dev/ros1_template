#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
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

    # This might break if current environment not setup
    # (catkin devel space sourced, pyros-setup config, or proper venv setup)
    import ros1_pip_pytemplate

    # We define the exception here instead of inside the library,
    # to keep side-effects outside of the core functionality
    # Another client of the library might want to treat status code differently,
    # but for ROS, treating non-OK status code as exception makes sense.
    class StatusCodeException(Exception):
        pass

    # Note : log_level here is about rosgraph logs, not python logging logs
    rospy.init_node(name, argv=argv)

    # retrieving ros parameters
    base_url = rospy.get_param("~base_url", None)

    httpbin = ros1_pip_pytemplate.Httpbin(base_url=base_url)

    # ROS environment is setup here
    import ros1_template_msgs.msg as ros1_template_msgs

    #
    # def callback(data):
    #     rospy.loginfo("received request:\n  {t}\n{v}".format(
    #         t=type(data),
    #         v="\n".join(["    " + l for l in str(data).splitlines()])))
    #     response = httpbin.get(params={a.key: a.value for a in data.args})
    #     if response.status_code == requests.status_codes.codes.OK:
    #         resp = ros1_template_srvs.GetResponse(
    #             origin=response.json().get('origin'),
    #             url=response.json().get('url'),
    #             args=[ros1_template_msgs.Arg(key=k, value=v)
    #                   for k, v in response.json().get('args').items()],
    #         )
    #         rospy.loginfo("sending response:\n  {t}\n{v}".format(
    #             t=type(resp),
    #             v="\n".join(["    " + l for l in str(resp).splitlines()])))
    #         return resp
    #     else:
    #         raise StatusCodeException(response.status_code)


    # setting up the publisher to the topic
    # This is not hte most appropriate interface for this, but it illustrate
    # the *other way* of handling communicating between two nodes.
    fibonacci_pub = rospy.Publisher('~get', ros1_template_msgs.Args, queue_size=1, latch=True)

    rate = rospy.Rate(1)  # 1Hz

    prev_webargs={}

    try:
        # Just spin and publish for ever, be proactive !
        while not rospy.is_shutdown():
            # getting our params
            rosargs = rospy.get_param('~args', {})
            # calling httpbin via proxy class:
            response = httpbin.get(params={k: v for k, v in rosargs.items()})
            if response.status_code == requests.codes.OK:
                # retrieving the returned args
                webargs = response.json().get("args")
                if webargs != prev_webargs:
                    # building the message we need to broadcast
                    fib_number = [ros1_template_msgs.Arg(key=k, value=v) for k, v in webargs.items()]
                    # logging it first
                    rospy.loginfo("publishing : {0}".format(fib_number))
                    # publishing it
                    fibonacci_pub.publish(ros1_template_msgs.Args(fib_number))
                    prev_webargs = webargs
            # sleeping a bit to not burn the CPU
            rate.sleep()

    except Exception as exc:
        rospy.logwarn("{0} detected in node {1} : {2}".format(type(exc), rospy.get_name(), str(exc)))
    finally:
        rospy.logwarn("{0} is shutting down !".format(rospy.get_name()))


##############################################################################
# Main
##############################################################################


def show_description():
    return "ros template client test script"


def show_usage(cmd):
    cmd = os.path.relpath(sys.argv[0], os.getcwd()) if cmd is None else cmd
    return "{0} [-h|--help] [--version] [--base_url <url_string>] ".format(cmd)


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

    if parsed_known_args.version:
        print("ROS1 pip pytemplate version " + ros1_pip_pytemplate.__version__ +
              "\n from " + ros1_pip_pytemplate.__file__)
        sys.exit(0)

    if parsed_known_args.base_url:
        # Here we change the standard arguments syntax to match ROS syntax, and we'll let rospy.init_node() do its magic
        sys.argv.remove('--base_url'), sys.argv.remove(parsed_known_args.base_url)
        sys.argv.append('_base_url' + ':=' + parsed_known_args.base_url)

    # We can now init the node (a ROS node is a process - this process -, that is an instance of the python interpreter)
    node_spin('httpbin_proactive', sys.argv)

    # TODO : ddynamice_reconfigure for argument to send to httpbin webservice

