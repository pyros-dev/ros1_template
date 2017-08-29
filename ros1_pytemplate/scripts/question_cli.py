#!/usr/bin/env python
#
# License: MIT
#

from __future__ import absolute_import, division, print_function

##############################################################################
# Imports
##############################################################################
import os
import sys
import argparse
import ros1_pytemplate

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
                'filename': 'ros1_pytemplate.log',
                'maxBytes': 1024,
                'backupCount': 3,
                'formatter': 'verbose'
            },
        },
        'loggers': {
            'ros1_template': {
                'handlers': ['logfile'],
                'level': 'DEBUG',
                'propagate': True,
            },
            'question': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
        }
    }
)


def show_description():
    return "ros template test script"


def show_usage(cmd=None):
    cmd = os.path.relpath(sys.argv[0], os.getcwd()) if cmd is None else cmd
    return "{0} [-h|--help] [--version]".format(cmd)


def show_epilog():
    return "never enough testing"

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
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

    logger = logging.getLogger("question")
    answer = ros1_pytemplate.Answer(6)
    logger.info(answer.retrieve())
