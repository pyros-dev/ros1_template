#!/usr/bin/env python
#
# License: MIT
#

from __future__ import absolute_import, division, print_function

##############################################################################
# Imports
##############################################################################
import argparse
import os
import sys

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
                'handlers': ['logfile'],
                'level': 'DEBUG',
                'propagate': True,
            },
            'cli': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
        }
    }
)


import ros1_pip_pytemplate

# for k in vars(ros1_pip_pytemplate):
#     print(k + ": " + str(getattr(ros1_pip_pytemplate, k)))


##############################################################################
# Main
##############################################################################


def show_description():
    return "Command line interface to the ros1_pip_pytemplate library"


def show_usage(cmd=None):
    cmd = sys.argv[0] if cmd is None else cmd
    return "{0} [-h] [--version] [--<arg> <value>] ... [--<arg> <value>] ".format(cmd)


def show_epilog():
    return "never enough testing"


def show_version():
    return ros1_pip_pytemplate.__version__, ros1_pip_pytemplate.__file__

if __name__ == '__main__':

    logger = logging.getLogger("cli.httpbin")

    # Ref : https://docs.python.org/2/library/argparse
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(),
                                     epilog=show_epilog(),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("--version", action='store_true', help="display the version number and exits.")

    parsed_known_args, unknown_args = parser.parse_known_args(sys.argv[1:])

    logger.info("known args: {0}".format(parsed_known_args))

    if parsed_known_args.version:
        print(show_version())
        sys.exit(0)

    # extracting argument from unknown args
    # Ref : https://stackoverflow.com/questions/9643248/argparse-accept-everything
    d = {}
    for arg in unknown_args:
        if arg.startswith('--'):  # O
            opt = arg[2:]
            d[opt] = None
        else:  # V
            try:
                # Note we authorize multiple value for one argument (will be changed into a list)
                if d[opt] is not None:
                    d[opt] = list(d[opt])
                    d[opt].append(arg)
                else:
                    d[opt] = arg
            except NameError:
                logger.warning("argument ignored (not related to option starting with '--') : {0}".format(arg))

    with_vals = {k: v for k, v in d.items() if v}
    without_vals = [k for k, v in d.items() if not v]

    logger.info("args with vals : {0}".format(with_vals))
    logger.info("args without vals : {0}".format(without_vals))

    httpbin = ros1_pip_pytemplate.Httpbin()
    response = httpbin.get(params=with_vals)
    logger.info(" {0} {1}".format(response.status_code, response.json()))
