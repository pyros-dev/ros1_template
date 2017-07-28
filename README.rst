ROS1 Template
=============

This package is a template to be used as a reference when creating a new ROS1 node to be used with catkin.
In this package we are following http://wiki.ros.org/BestPractices as much as possible, where it makes sense.

Usage
-----

- Create a catkin_workspace ( Ref : http://wiki.ros.org/catkin/Tutorials/create_a_workspace )
- Add this ros1_template pacakge to the workspace (and maybe more like beginner_tutorials)
- Build the workspace (Ref : http://wiki.ros.org/catkin/Tutorials/using_a_workspace
- Source the devel workspace::

  source devel/setup.bash

- manually run some python test::

  nosetests ros1_template

- manually run some ROS tests::

  rostest ros1_template

- run all the tests::

  catkin run_tests

- launch some nodes::

  roslaunch ros1_template --screen

- install that workspace::

  make install

- source the install workspace::

  source devel/setup.bash

- launch some nodes::

  roslaunch ros1_template --screen




