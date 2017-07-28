#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#

""" Testing the find_node function """

##############################################################################
# Imports
##############################################################################

import rocon_console.console as console
import rocon_app_manager
import rospy
import rostest
import unittest
import tempfile

##############################################################################
# Test Class
##############################################################################


class TestRemappings(unittest.TestCase):
    def test_apply_remapping_rules(self):

if __name__ == '__main__':
    rospy.init_node("test_remappigns")
    rostest.rosrun('rocon_app_manager',
                   'test_remappings',
                   TestRemappings)