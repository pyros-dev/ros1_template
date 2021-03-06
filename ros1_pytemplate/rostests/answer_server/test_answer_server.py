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

import rospy
import rostest
import ros1_template_msgs.srv as ros1_template_srvs


##############################################################################
# Test Class
##############################################################################


class TestAnswerServer(unittest.TestCase):

    def test_answer_service(self):
        rospy.wait_for_service('~answer')
        try:
            answer_proxy = rospy.ServiceProxy('~answer', ros1_template_srvs.Answer)
            req = ros1_template_srvs.AnswerRequest('What is the Answer to the Ultimate Question of Life, the Universe and Everthing ?')
            resp = answer_proxy(req)
            self.assertEqual(resp.answer, 42)
        except rospy.ServiceException as exc:
            print("service call failed: {0}".format(exc))
            raise

    def test_error_service(self):
        rospy.wait_for_service('~error')
        with self.assertRaises(rospy.ServiceException):
            error_proxy = rospy.ServiceProxy('~error', ros1_template_srvs.Answer)
            req = ros1_template_srvs.AnswerRequest('This works, right ?')
            error_proxy(req)


if __name__ == '__main__':
    # Note : logging is managed by rostest
    print("ARGV : {0}".format(sys.argv))
    rospy.init_node('test_answer_server')  # mandatory for using topics
    rostest.rosrun('ros1_pytemplate', 'test_answer_server', TestAnswerServer, sys.argv)
