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
import rosgraph_msgs.msg as rosgraph_msgs

##############################################################################
# Test Class
##############################################################################


# test this with:
# > rostest ros1_template test_follower.test
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

        self.fib_pub.publish(21)

        # wait for a message
        now = time.time()
        # five seconds max
        while time.time() - now < 5 and len(self.follower_logmsg) < 1:
            time.sleep(.1)

        # asserting the log output
        self.assertTrue(len(self.follower_logmsg) >= 1)
        self.assertEqual("The next Fibonacci Number was heard: 21", self.follower_logmsg[-1])

# run this with:
# > rosrun ros1_template test_follower.py
if __name__ == '__main__':
    # rostest does this for you
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # Same as <node pkg="ros1_template" type="follower_node.py" name="follower"/> in .test file
    # Ref : http://docs.ros.org/indigo/api/roslaunch/html/index.html
    follower = roslaunch.core.Node('ros1_pytemplate', 'fibonacci_sub_node.py', name='fibonacci_sub')
    follower_proc = launch.launch(follower)
    assert follower_proc.is_alive()

    # Same as <test test-name="test_follower" pkg="ros1_template" type="test_follower.py"/> in .test file
    rospy.init_node("test_fibonacci_sub")
    test_result = rostest.rosrun('ros1_pytemplate', 'test_fibonacci_sub', TestFibonacciSub)

    # cleaning up
    follower_proc.stop()
    launch.stop()

    sys.exit(test_result)
