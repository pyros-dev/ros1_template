<!--- Kind of hackish way to make it work on github and Doxygen) -->

[//]: # (/*! \mainpage ( ROS1 CPP Template */)

ROS1 CPP Template
=================

This package is a template to be used as a reference when creating c++ libraries (and applications if wanted) to be used in a ROS package. The main part of the implementation should go into this package and be independent from ROS.

The only packge dependencies are on the *roslint* for checking the coding style (build dependency only) and *rosconsole* to have a uniform way for logging. In this package we are following http://wiki.ros.org/BestPractices as much as possible, where it makes sense.

Usage
-----

- Create a catkin_workspace ( Ref : http://wiki.ros.org/catkin/Tutorials/create_a_workspace )
- Add this ros1_cpptemplate package into ``src/`` of the workspace (and maybe more like beginner_tutorials)
- Build the workspace (Ref : http://wiki.ros.org/catkin/Tutorials/using_a_workspace)
- Source the devel workspace::

    $ source devel/setup.bash

- run the tests of this package::

    $ catkin run_tests ros1_cpptemplate

- run all the tests::

    $ catkin run_tests

- launch the demo application::

    $ ./devel/lib/ros1_cpptemplate/multithread_fibonacci

- install that workspace::

    $ make install

- source the install workspace::

    $ source install/setup.bash




