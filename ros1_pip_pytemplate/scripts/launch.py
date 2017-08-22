from __future__ import absolute_import, division, print_function

import time

"""
Launch python script, for when the inflexible ROS1 xml-based launch files are just not enough...
"""

import roslaunch

# This should have the same effect as any <package_name>.launch file for this package.

if __name__ == '__main__':
    # Start roslaunch
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    global httpbin_process

    httpbin_node = roslaunch.core.Node('httpbin', 'httpbin.py', name='httpbin')
    try:
        httpbin_process = launch.launch(httpbin_node)
    except roslaunch.RLException as rlexc:
        raise

    assert httpbin_process.is_alive()

    while launch.is_alive():
        time.sleep(1)

    # ensuring launched process are finished
    if httpbin_process.is_alive():
        httpbin_process.stop()
