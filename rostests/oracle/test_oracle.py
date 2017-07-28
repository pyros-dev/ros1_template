#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#
""" Testing the oracle """

##############################################################################
# Imports
##############################################################################

import rospy
import rostest
import unittest
import tempfile
import ros1_template.msg as ros1_template_msgs
import ros1_template.srv as ros1_template_srvs


##############################################################################
# Test Class
##############################################################################


class TestOracle(unittest.TestCase):

    def test_answer_service(self):
        rospy.wait_for_service('/oracle/answer')
        try:
            answer_proxy = rospy.ServiceProxy('/oracle/answer', ros1_template_srvs.Answer)
            req = ros1_template_srvs.AnswerRequest('What is the Answer to the Ultimate Question of Life, the Universe and Everthing ?')
            resp = answer_proxy(req)
            print(resp)
            self.assertEqual(resp.answer, 42)
        except rospy.ServiceException as exc:
            print("service call failed: {0}".format(exc))

    def test_error_service(self):
        rospy.wait_for_service('/oracle/error')
        with self.assertRaises(rospy.ServiceException):
            error_proxy = rospy.ServiceProxy('/oracle/error', ros1_template_srvs.Answer)
            req = ros1_template_srvs.AnswerRequest('This works, right ?')
            error_proxy(req)


if __name__ == '__main__':
    rospy.init_node("test_oracle")
    rostest.rosrun('ros1_template', 'test_oracle', TestOracle)
