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
import rosgraph_msgs.msg as rosgraph_msgs

##############################################################################
# Test Class
##############################################################################


class TestFibonacciSub(unittest.TestCase):

    def setUp(self):
        self.fib_pub = rospy.Publisher('/fibonacci_sub/fibonacci', ros1_template_msgs.Fibonacci)

        self.follower_logmsg = []

        def logcb(data):
            if data.name == '/fibonacci_sub':
                self.follower_logmsg += [data.msg]

        # We re listening on the log topic to assert the follower did its job.
        self.logsub = rospy.Subscriber('/rosout', rosgraph_msgs.Log, logcb)

    def tearDown(self):
        pass

    def test_follower(self):
        # waiting for at least one subscriber (to make sure we don't lose the first message)
        now = time.time()
        # five seconds max
        while time.time() - now < 5 and self.fib_pub.get_num_connections() < 1:
            time.sleep(.1)

        # make sure we didnt timeout
        self.assertTrue(self.fib_pub.get_num_connections() >= 1)

        self.fib_pub.publish(21)

        # wait for a message
        now = time.time()
        # five seconds max
        while time.time() - now < 5 and len(self.follower_logmsg) < 1:
            time.sleep(.1)

        # make sure we didn't timeout
        self.assertTrue(len(self.follower_logmsg) >= 1)

        # asserting the log output
        self.assertTrue(len(self.follower_logmsg) >= 1)
        self.assertEqual("The next Fibonacci Number was heard: 21", self.follower_logmsg[-1])

# run this with:
# > rostest ros1_template fibonacci_sub.test
if __name__ == '__main__':
    # note : logging is managed by rostest
    print("ARGV : {0}".format(sys.argv))
    rospy.init_node('test_fibonacci_sub')  # mandatory for using topics
    rostest.rosrun('ros1_pytemplate', 'test_fibonacci_sub', TestFibonacciSub, sys.argv)

