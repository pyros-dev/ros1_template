^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_pip_pytemplate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fixing tests because url arguments can come in different order...
* Merge pull request `#17 <https://github.com/pyros-dev/ros1_template/issues/17>`_ from pyros-dev/pip_template
  Adding catkin pip template
* adding missing ros dependencies.
* fixing node init in tests to work for both rostest and pytest.
* not requiring latest catkin_pip
* tox tests passing (non-ROS)
* python tests passing.
* proactive node test passing
* proactive node working.
* fixing catkin tests
* getting reactive node tests to pass without calling rospy.init_node
* implemented test_cli
* WIP reviewing cli and node script and tests
* now unittesting logger output.
* added version argument for cli
* cleaning up and start setting up tox
* dropping the python launcher idea here, lets stay close to catkin_pip style packages.
  They are python packages ported to ROS, and integrated in ROS systems via launch files and yaml config.
* getting httpbin proxy ros node to work
* WIP template copy pasted code
* Merge branch 'master' into fix_launchers
* Merge pull request `#7 <https://github.com/pyros-dev/ros1_template/issues/7>`_ from pyros-dev/python
  restructuring and renaming packages
* fixing ros1_pip_template -> ros1_pip_pytemplate
* Merge pull request `#8 <https://github.com/pyros-dev/ros1_template/issues/8>`_ from pyros-dev/python-pip
  adding template to use catkin_pip to get python package in catkin wor…
* renaming pip package, fixed install, now using docker images.
* adding template to use catkin_pip to get python package in catkin workspace.
* Contributors: AlexV, Alexander Reimann
