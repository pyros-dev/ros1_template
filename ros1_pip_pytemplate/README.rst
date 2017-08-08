ROS1 Template
=============

.. image:: https://travis-ci.org/pyros-dev/ros1-pip-template.svg?branch=master
    :target: https://travis-ci.org/pyros-dev/ros1-pip-template


This package is a template to be used as a reference when creating a new ROS1 node, based on python.
We are relying on catkin_pip, to provide pip integration in catkin and make it easy to use python package in ROS *source* package.
In this package we are following http://packaging.python.org/ as much as possible, where it makes sense.
Note also that this package is python3 ready.

Following this template allows to integrate ROS and python quite closely, and can be used when undergoing a ROS -> python transition or vice-versa.
See https://github.com/pyros-dev/catkin_pip for more information.

Usage from catkin
-----------------

- Create a catkin_workspace ( Ref : http://wiki.ros.org/catkin/Tutorials/create_a_workspace )
- Add this ros1_pip_pytemplate package to the workspace (and maybe more like ros1_template and beginner_tutorials)
- Build the workspace (Ref : http://wiki.ros.org/catkin/Tutorials/using_a_workspace
- Source the devel workspace::

    $ source devel/setup.bash

- manually run some python test::

    $ pytest tests/test_ros1_template/test_lib_module.py

- run all the tests::

    $ catkin run_tests

- launch some nodes (log on terminal)::

    $ roslaunch ros1_template all.launch --screen

- install that workspace::

    $ make install

- source the install workspace::

    $ source install/setup.bash

- run all the tests from the package::

    $ catkin run_tests

- launch some nodes (log in files in .ros/log)::

    $ roslaunch ros1_template all.launch

To create a ROS package from this template you should follow http://wiki.ros.org/bloom


Usage from python
-----------------

- Create a virtualenv (using virtualenvwrapper)::

    $ mkvirtualenv template_venv

- Make sure you are using the latest pip::

    (template_venv)$ pip install pip --upgrade

- Install the package as editable::

    (template_venv)$ pip install -e .

- run the tests from source::

    (template_venv)$ pytest tests/

- run the tests from package::

    (template_venv)$ pytest --pyargs ros1_pip_pytemplate.tests

To create a pip package from this template, you should follow https://packaging.python.org/tutorials/distributing-packages

Note that this template can be used when porting existing pip packages to ROS.
More documentation can be found at http://docs.ros.org/kinetic/api/catkin_pip/html/