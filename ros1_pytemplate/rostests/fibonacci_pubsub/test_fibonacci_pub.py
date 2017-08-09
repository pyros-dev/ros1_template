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
import roslaunch
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
# > rosrun ros1_template test_prophet.py
if __name__ == '__main__':
    # rostest does this for you
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # Same as <param name="fib_init" value="[0, 1]"/> in .test file
    rospy.set_param("/fibonacci_pub/fib_init", [0, 1])

    # Same as <node pkg="ros1_template" type="prophet_node.py" name="prophet"/> in .test file
    # Ref : http://docs.ros.org/indigo/api/roslaunch/html/index.html
    follower = roslaunch.core.Node('ros1_pytemplate', 'fibonacci_pub_node.py', name='fibonacci_pub')
    follower_proc = launch.launch(follower)
    assert follower_proc.is_alive()

    # Same as <test test-name="test_prophet" pkg="ros1_template" type="test_prophet.py"/> in .test file
    rospy.init_node("test_fibonacci_pub")  # mandatory for using topics
    test_result = rostest.rosrun('ros1_pytemplate', 'test_fibonacci_pub', TestFibonacciPub)

    # cleaning up
    follower_proc.stop()
    launch.stop()

    sys.exit(test_result)
