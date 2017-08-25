#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#

from __future__ import absolute_import, division, print_function

import time

"""
Testing the node

This module can however be run directly from a python virtual env
where pyros_setup and pyros_utils have been installed and configured
pyros_setup will take care of setting up this process environment for ROS.
"""

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

# Note: Here we want to test the logger
# Not the terminal output, since this will depend on the top level launcher)

httpbin_proc = None


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # rostest does this for you
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # Same as <node pkg="ros1_pip_pytemplate" type="node.py" name="httpbin">
        #   <param name="base_url" value="http://httpbin.org"/>
        # </node>
        # Ref : http://docs.ros.org/indigo/api/roslaunch/html/index.html
        rospy.set_param('/httpbin/base_url', "http://httpbin.org")
        httpbin = roslaunch.core.Node('ros1_pip_pytemplate', 'node_reactive.py', name='httpbin')
        httpbin_proc = launch.launch(httpbin)
        assert httpbin_proc.is_alive()

        # WE DO NOT NEED A NODE for sending requests to services
        # But this is where we would do it...
        # rospy.init_node('test_httpbin')
        # CAREFUL : this should be done only once per PROCESS
        # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()
        if httpbin_proc and httpbin_proc.is_alive():
            httpbin_proc.terminate()


class TestHttpbinProxy(unittest.TestCase):

    def test_httpbin_get(self):
        rospy.wait_for_service('/httpbin/get')
        try:
            get_proxy = rospy.ServiceProxy('/httpbin/get', ros1_template_srvs.Get)
            req = ros1_template_srvs.GetRequest(args=[])
            resp = get_proxy(req)
            self.assertEqual(resp.url, "http://httpbin.org/get")
            self.assertEqual(resp.args, [])
        except rospy.ServiceException as exc:
            print("service call failed: {0}".format(exc))


if __name__ == '__main__':
    print("ARGV : {0}".format(sys.argv))
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('ros1_pip_pytemplate', 'test_httpbin', TestHttpbinProxy, sys.argv)
