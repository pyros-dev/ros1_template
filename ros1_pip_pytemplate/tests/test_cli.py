#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#

from __future__ import absolute_import, division, print_function

"""
Testing the cli
"""

##############################################################################
# Imports
##############################################################################

import sys
import unittest
import tempfile

import rostest
import roslaunch
import ros1_template_msgs.msg as ros1_template_msgs
import ros1_template_msgs.srv as ros1_template_srvs

from pyros_utils import rostest_nose


##############################################################################
# Test Class
##############################################################################

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
