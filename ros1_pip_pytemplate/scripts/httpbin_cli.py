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
                'filename': 'ros1_pip_template.log',
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


import ros1_pip_template
for k in vars(ros1_pip_template):
    print(k + ": " + str(getattr(ros1_pip_template, k)))
##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    logger = logging.getLogger("question")
    answer = ros1_pip_template.Httpbin()
    logger.info(ros1_pip_template.Httpbin.get())
