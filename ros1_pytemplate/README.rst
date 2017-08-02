ROS1 Python Template
====================

This package is a template to be used as a reference when creating a new ROS1 node with python, to be used with catkin.
In this package we are following http://wiki.ros.org/BestPractices as much as possible, where it makes sense.

Usage
-----

- Create a catkin_workspace ( Ref : http://wiki.ros.org/catkin/Tutorials/create_a_workspace )
- Add this ros1_pytemplate package into ``src/`` of the workspace (and maybe more like beginner_tutorials)
- Build the workspace (Ref : http://wiki.ros.org/catkin/Tutorials/using_a_workspace
- Source the devel workspace::

    $ source devel/setup.bash

- manually run some python test::

    $ nosetests tests/test_ros1_pytemplate/test_lib_module.py

- manually run some ROS tests::

    $ rostest ros1_pytemplate oracle.test

- run all the tests::

    $ catkin run_tests

- launch some nodes (log on terminal)::

    $ roslaunch ros1_pytemplate all.launch --screen

- install that workspace::

    $ make install

- source the install workspace::

    $ source install/setup.bash

- launch some nodes (log in files in .ros/log)::

    $ roslaunch ros1_pytemplate all.launch




