from __future__ import absolute_import, division, print_function

"""
Launch python script, for when the inflexible ROS1 xml-based launch files are just not enough...
"""

import roslaunch

# This should have the same effect as any <package_name>.launch file for this package.

if __name__ == '__main__':
    # Start roslaunch
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # start required nodes - needs to match the content of *.test files for rostest to match

    global httpbin_process

    httpbin_node = roslaunch.core.Node('pyros_httpbin', 'httpbin.py', name='httpbin')
    try:
        httpbin_process = launch.launch(httpbin_node)
    except roslaunch.RLException as rlexc:
        raise

    # finishing all process are finished
    if httpbin_process is not None:
        httpbin_process.stop()

    launch.stop()
