#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#

""" Testing the find_node function """

##############################################################################
# Imports
##############################################################################

import sys
import unittest
import time

import rospy
import rostest
import ros1_template_msgs.msg as ros1_template_msgs

##############################################################################
# Test Class
##############################################################################


class TestFibonacciPub(unittest.TestCase):
    def setUp(self):
        self.fibnumbers = []

        def fibcb(data):
            self.fibnumbers += [data.number]

        self.fibsub = rospy.Subscriber('/fibonacci_pub/fibonacci', ros1_template_msgs.Fibonacci, fibcb)

    def tearDown(self):
        pass

    def test_prophet(self):

        # wait for a message
        now = time.time()
        # five seconds max
        while time.time() - now < 5 and len(self.fibnumbers) < 3:
            time.sleep(.1)

        # asserting fibonacci's sequence
        self.assertTrue(len(self.fibnumbers) >= 3)
        self.assertEqual(self.fibnumbers[2], self.fibnumbers[0] + self.fibnumbers[1])


# run this with:
# > rostest ros1_template fibonacci_pub.test
if __name__ == '__main__':
    # note : logging is managed by rostest
    print("ARGV : %r", sys.argv)
    test_result = rostest.rosrun('ros1_pytemplate', 'test_fibonacci_pub', TestFibonacciPub, sys.argv)
    sys.exit(test_result)
