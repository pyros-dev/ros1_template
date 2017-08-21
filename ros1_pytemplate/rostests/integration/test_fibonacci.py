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
import rosgraph_msgs.msg as rosgraph_msgs

##############################################################################
# Test Class
##############################################################################


class TestFibonacci(unittest.TestCase):
    def setUp(self):
        self.sub_logmsg = []

        def logcb(data):
            if data.name == '/fibonacci_sub':
                self.sub_logmsg += [data.msg]

        # We re listening on the log topic to assert the follower did its job.
        self.logsub = rospy.Subscriber('/rosout', rosgraph_msgs.Log, logcb)

    def tearDown(self):
        pass

    def test_fibonacci(self):
        # wait for a message
        now = time.time()
        # five seconds max
        while time.time() - now < 5 and len(self.sub_logmsg) < 1:
            time.sleep(.1)

        # Here we only assert that we get the expected log (we do not check if fib is actually a fib
        # since it should be already done by fibonacci_pub rostest
        self.assertTrue(len(self.sub_logmsg) >= 1)
        self.assertTrue("The next Fibonacci Number was heard" in self.sub_logmsg[-1])


# run this with:
# > rostest ros1_template fibonacci_pubsub.test
if __name__ == '__main__':
    # note : logging is managed by rostest
    print("ARGV : %r", sys.argv)
    test_result = rostest.rosrun('ros1_pytemplate', 'test_fibonacci', TestFibonacci, sys.argv)
    sys.exit(test_result)
