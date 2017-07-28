#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#

""" Testing the find_node function """

##############################################################################
# Imports
##############################################################################

import rospy
import rostest
import unittest

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