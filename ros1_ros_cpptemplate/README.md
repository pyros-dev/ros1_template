<!--- Kind of hackish way to make it work on github and Doxygen) -->

[//]: # (/*! \mainpage ( ROS1 CPP Template */)

ROS1 CPP Template
=================

This package is a template for a ROS nodelet package depending on another C++ library package for implementation details.
It is supposed to show the ROS part if using a separate ROS and C++ library packages.
In this package we are following http://wiki.ros.org/BestPractices as much as possible, where it makes sense.

Usage
-----

- Create a catkin_workspace ( Ref : http://wiki.ros.org/catkin/Tutorials/create_a_workspace )
- Add this ros1_cpptemplate package into ``src/`` of the workspace (and maybe more like beginner_tutorials)
- Build the workspace (Ref : http://wiki.ros.org/catkin/Tutorials/using_a_workspace)
- Source the devel workspace::

    $ source devel/setup.bash

- run the tests of this package::

    $ catkin run_tests ros1_ros_cpptemplate

- run all the tests::

    $ catkin run_tests

- launch the demo application::

    $ roslaunch ros1_ros_template standalone.launch --screen

- install that workspace::

    $ make install

- source the install workspace::

    $ source install/setup.bash




