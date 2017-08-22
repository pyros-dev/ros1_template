#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#
""" Testing the oracle """

##############################################################################
# Imports
##############################################################################

import sys
import unittest
import tempfile

# Doing ROS setup from code if required
try:
    import rospy
except ImportError:
    import pyros_setup
    pyros_setup.configurable_import().configure().activate()
    import rospy

import rospy
import rostest
import roslaunch
import ros1_template_msgs.msg as ros1_template_msgs
import ros1_template_msgs.srv as ros1_template_srvs

from pyros_utils import rostest_nose


##############################################################################
# Test Class
##############################################################################

oracle_proc = None

# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # rostest does this for you
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # Same as <node pkg="ros1_pytemplate" type="answer_server_node.py" name="answer_server">
        #            <param name="answer_part" value="6"/>
        #         </node> in .test file
        # Ref : http://docs.ros.org/indigo/api/roslaunch/html/index.html
        rospy.set_param('/answer_server/answer_part', 6)
        oracle = roslaunch.core.Node('ros1_pytemplate', 'answer_server_node.py', name='answer_server')
        oracle_proc = launch.launch(oracle)
        assert oracle_proc.is_alive()

        # No need of a node for using services, but this is what rostest does under the hood.
        rospy.init_node('test_answer_server', anonymous=True, disable_signals=True)
        # CAREFUL : this should be done only once per PROCESS
        # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()
        if oracle_proc and oracle_proc.is_alive():
            oracle_proc.terminate()


class TestAnswerServer(unittest.TestCase):

    def test_answer_service(self):
        rospy.wait_for_service('/answer_server/answer')
        try:
            answer_proxy = rospy.ServiceProxy('/answer_server/answer', ros1_template_srvs.Answer)
            req = ros1_template_srvs.AnswerRequest('What is the Answer to the Ultimate Question of Life, the Universe and Everthing ?')
            resp = answer_proxy(req)
            print(resp)
            self.assertEqual(resp.answer, 42)
        except rospy.ServiceException as exc:
            print("service call failed: {0}".format(exc))

    def test_error_service(self):
        rospy.wait_for_service('/answer_server/error')
        with self.assertRaises(rospy.ServiceException):
            error_proxy = rospy.ServiceProxy('/answer_server/error', ros1_template_srvs.Answer)
            req = ros1_template_srvs.AnswerRequest('This works, right ?')
            error_proxy(req)


if __name__ == '__main__':
    print("ARGV : %r", sys.argv)
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    #rostest_nose.rostest_or_nose_main('ros1_pytemplate', 'test_answer_server', TestAnswerServer, sys.argv)
    rostest.rosrun('ros1_pytemplate', 'test_answer_server', TestAnswerServer, sys.argv)
