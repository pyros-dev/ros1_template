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

import ros1_template

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    print(os.getcwd())
    ros1_template(args=sys.argv)  # this is interactive because it comes from command line)
