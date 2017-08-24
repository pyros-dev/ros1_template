ROS1 Presentation
=================

Contents:
- Why ROS
- ROS Basics
- Mixing C++ and Python
- Distributed Robot System
- Testing and Safety Considerations


---

Why ROS ?
=========

- A set of tools for C++
- A set of components for Robots
- Big Robotics Community
- Permissive Licensing


+++

A set of C++ tools
------------------

C++ does NOT have de facto tools (like python)

ROS provides : 
- catkin => CMake made easy
- rosdoc => doxygen made easy
- rostest => gtest made easy
- ROSTCP => networking made easy
- rosmsg => serialization made easy
- etc.

+++

A set of components for robots
------------------------------

- calibration tools
- development tools
- visualisaton
- navigation
- manipulation
- ...

+++

Big community
-------------

- ROS wiki page views : 1,394,363
- International : US, CN, JP, DE > 25,000
- ROS wiki users : 5875
- ROS pkg downloads : 8,441,279 from 113,000 IPs
- ROS research papers : 2683
- ROS Robots : > 100 (source : http://wiki.ros.org/Robots)

source : http://wiki.ros.org/Metrics

+++

Permissive licensing
--------------------

- ROS core BSD license (Free to use, modify, sell)
- ROS tools: Open Source/Free licenses (free to use)



---

ROS1 Basics : Do the tutorials !
================================

+++

ROS1 Environment
----------------

`$ source setup.bash`
- modify your current SHELL (`bash`) environment.
- GOOD: easy to modify quickly (`export MY_ROS_ENVVAR=42`)
- BAD : hard to debug (`ps aux | grep my_ros_program`) => which environment configuration ?

+++

ROS1 Filesystem
---------------

`rosls`, `roscd`, `rospack`, etc.
- allows easy navigation in ROS environment
- GOOD: easy to play around in your ROS environment
- BAD : not fully isolated from the rest of the system

+++

ROS1 Packages
-------------

`package.xml` and `CMakeLists.txt`
- describes a ROS package
- GOOD: modular compilable source code (`catkin_make` or `catkin build`)
- GOOD: modular interactive runtime (ROS API)
- BAD : not easy to compose, easy to shoot yourself
- GOOD: integrating with your package manager
- BAD : not quite integrated, it goes into `/opt/ros/<rosdistro>`

+++

ROS1 Dependencies
-----------------

`rosdep resolve depkey`

- enable cross-platform package management
- GOOD: can edit github's `ros/rosdistro` repository to add dependencies 
- BAD : currently can confuse ROS packages and dependencies (BUG)

+++

ROS1 Nodes
----------

`rospy.init_node()` or `ros::init()`
- declares the current process as a ROS node
- GOOD: one node match one process
- BAD : special `rosmaster`, `rosout` nodes
- BAD : pollute your process global state
- BAD : async programming can be VERY confusing
C++ has nodelets, which means that one process has N node(let)s

+++

ROS1 Launchers
--------------

`roslaunch my_pkg my_nodes.launch`

- provides a centralised launch point for multiple process/nodes
- GOOD: can load yaml files to set parameters for the system
- GOOD: a launch file can include another
- BAD : namespaces logic and remapping is not as simple as it could 
- BAD : XML, but more programming-style logic is needed

+++

ROS1 Parameters
---------------

`$ rosparam get` or `rospy.get_param()` or `ros::NodeHandle::getParam()`

- get/set parameters for the running ROS system
- GOOD: centralised structured data
- GOOD: dynamically configurable 

+++

ROS1 Messages
-------------

`$ rosmsg show`

- defines message structure
- GOOD: provide an API other can use
- GOOD: provides serialization/deserialization crossplatform
- BAD : doesnt always check and validate the data type

+++

ROS1 Topics
-----------

`$ rostopic pub` or `rospy.Publisher()` or `ros::Publisher`

`$ rostopic echo` or `rospy.Subscriber()` or `ros::Subscriber`

- allow ROSTCP/ROSUDP Push communication / event notifications
- GOOD: low overhead (binary serialization)
- BAD : lossy
- BAD : Inverted control flow (deal with your callback threadS)
- BAD : Cannot be used to interface with non-ROS systems
- GOOD : need 2 to implement pull communication

+++

ROS1 Services
-------------

`rospy.Service()` or `ros::Service`

`$ rosservice call` or `rospy.ServiceProxy()` or `ros::ServiceProxy`

- allow XMLRPC Pull communication / Remote Procedure Calls
- BAD : high overhead (XML)
- GOOD: lossless
- GOOD: Direct control flow (deal with your main thread)
- GOOD: CAN be used to interface with non-ROS systems
- GOOD: need 2 to implement push communication

---

Mixing C++ and Python
=====================

- Different tools, standards, community, mindset. Pick a side !
- C/C++ and Python can mix in different ways (cffi, bindings, ...)
- ROS provide the multi-process/local-network way.
- ROS takes its C++ tools and standards and adapt them (more or less) to python.

+++

C++
---

- Build : GNU make, CMake, QMake, Ninja, Gradle, SCons, Premake, etc.
- Docs  : Doxygen, Sphinx, etc.
- Tests : Boost.Test, cppunit, CxxTest, gtest, etc.
- Logs  : Boost.Log, log4cpp, log4cxx, etc.
- Style : astyle, uncrustify, etc.
- Analysis: cppcheck, cpplint, clang, etc.
- packaging: deb, rpm, CPack, etc. 

+++

Python
------

- Build : Python
- Docs  : Sphinx
- Tests : unitest, doctest, nose, pytest
- Logs  : logging
- Style : PEP8
- Analysis: mypy
- Packaging: pip + setuptools

+++

Static
------

- Check what you can
- Compile what you need
- Don't change
- Be fast

+++

Dynamic
-------

- Check if you can
- Compile if you need
- Change if you need
- Be open

+++

Python with C/C++ extensions
----------------------------

- Python API
- CPython interface
- CFFI
- documented & standardised

+++

The ROS way
-----------

- communicating asynchronous processes
- common message format
- compatible serialization/deserialization code
- basic network management code


---

Distributed Robot System
========================

- Distributed Software System is hard (heavily researched software topic)
- Forget what you think you know.
- Learn erlang.
- Murphy's Law: Everything that can possibly go wrong WILL go wrong.
- But Robot system must be reliable (enough) ?!?!


+++

Distributed Software System : time is relative
----------------------------------------------

This: `a=time.now()` `b=time.now()` `print(a>b)` can print:
- `True`
- `False`
- `no idea`

+++

Distributed Software System : no total order
--------------------------------------------

This: `int a=1` `a= a+1` `print(a)` can print:
- `1`
- `2`
- `42`
- ...
- `0`
- Error

+++

Distributed Software System : CRDT - 2011
-----------------------------------------

- G-counter: we can count up
- PN-counter: we can count up and down
- more to come


+++


Forget what you think you know
------------------------------


+++



Learn Erlang
------------

- 
