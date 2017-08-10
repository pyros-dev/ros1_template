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

# Integrating tests with PyCharm (setup the ROS environment)
# import pyros_setup
# pyros_setup.configurable_import().configure().activate()

import rospy
import rostest
import roslaunch
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
# > rosrun ros1_template test_prophet.py
if __name__ == '__main__':
    # rostest does this for you
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # Same as <param name="fib_init" value="[0, 1]"/> in .test file
    rospy.set_param("/fibonacci_pub/fib_init", [0, 1])

    # TODO : roslaunch API to start a launchfile instead of directly running a node

    # Same as <node pkg="ros1_template" type="fibonacci_pub_node.py" name="fibonacci_pub"/> in .test file
    # Ref : http://docs.ros.org/indigo/api/roslaunch/html/index.html
    pub = roslaunch.core.Node('ros1_pytemplate', 'fibonacci_pub_node.py', name='fibonacci_pub')
    pub_proc = launch.launch(pub)
    assert pub_proc.is_alive()

    # Same as <node pkg="ros1_template" type="fibonacci_sub_node.py" name="fibonacci_sub"/> in .test file
    # Ref : http://docs.ros.org/indigo/api/roslaunch/html/index.html
    sub = roslaunch.core.Node('ros1_pytemplate', 'fibonacci_sub_node.py', name='fibonacci_sub',
                              # remapping to connect sub to pub.
                              remap_args=[("/fibonacci_sub/fibonacci", "/fibonacci_pub/fibonacci")])
    sub_proc = launch.launch(sub)
    assert sub_proc.is_alive()

    # Same as <test test-name="test_prophet" pkg="ros1_template" type="test_prophet.py"/> in .test file
    rospy.init_node("test_fibonacci")  # mandatory for using topics
    test_result = rostest.rosrun('ros1_pytemplate', 'test_fibonacci', TestFibonacci)

    # cleaning up
    pub_proc.stop()
    sub_proc.stop()
    launch.stop()

    sys.exit(test_result)
