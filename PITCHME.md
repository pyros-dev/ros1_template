ROS1 Presentation
=================

Contents:
- Why ROS ?
- ROS Basics
- C++ and Python, pick one
- ROS exercises
- Distributed System for Robots
- Testing and Safety


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
- <span style="color:green">GOOD</span>: easy to modify quickly (`export MY_ROS_ENVVAR=42`)
- <span style="color:red">BAD</span> : hard to debug (`ps aux | grep my_ros_program`) => which environment configuration ?

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

ROS C++ Exercises
=================

- Make a C++ "package" without ROS
- Alex TODO

---

ROS Python Exercises
====================

- Make a ROS package
- Make a Python package
- Turn it into a ROS package
- Define Architecture

+++

Make a ROS package
------------------

- Follow ROS python tutorials
- Provide a Number Node that store a number (init at 1)
- with ROS API for reset() and value()


+++

Make a Python package
---------------------

- Follow http://packaging.python.org
- Provide a python package with Accumulator module
- Accumulator stores a number N (init at 1) and a constant C
- Accumulator can do inc_mod(m) -> (N + m) mod C

+++

Turn it into a ROS package
--------------------------

- Use catkin_pip
- Unify tests
- Make a ROS Accumulator node
- Choose a ROS API to provide access to inc_mod

+++

Make a client python script
---------------------------

- Not a node
- Uses Number & Accumulator ROS API
- Implements a Fibonacci modulo sequence
- Doesnt do any arithmetic (+, - *, /) itself

+++

Mutate client into a node
-------------------------

- Can we improve the communication
- Is it really better ?
- What are the problems or risks ? 

+++

Review
------

- We (just) implemented Fibonacci
- What can we say about the code ?
- What can we way about the potential bugs ?
- How can we explain it to someone else ? 

---


Distributed Robot System
========================

- Distributed Computing Fallacies
- Forget what you think you know
- Recent research area
- No mainstream distributed programming environment.
- Murphy's Law: Everything that can possibly go wrong WILL go wrong.
- But Robot system must be reliable (enough) ?!?!

+++

Distributed Computing Fallacies
-------------------------------

- the network is _reliable_
- latency is _zero_
- bandwith is _infinite_
- network is _secure_
- Topology _doesnt change_
- There is _one administrator_
- Transport cost is _zero_
- Network is _homogeneous_

+++

FWYTYK : time is relative
-------------------------

This: `a=time.now()` `b=time.now()` `print(a>b)` can print:
- `True`
- `False`
- `no idea`
- Error
- Crash

+++

FWYTYK : no total order
-----------------------

This: `int a=1` `a= a+1` `print(a)` can print:
- `1`
- `2`
- `42`
- ...
- `0`
- Error
- Crash

+++

Recent Research Area : CRDT - 2011
----------------------------------

- Grow-counter: we can count up !
- Positive-Negative-counter: we can count down !
- Grow-only-set: we can group things together !
- Two-phase-set: we can ungroup things !
- more coming...


+++


No mainstream distributed programming environment
-------------------------------------------------

- Except Erlang (31 years old)
- http://learnyousomeerlang.com/distribunomicon

```
(Alice@alexv-pc)1> net_kernel:connect_node('Bob@alexv-pc').
true
(Alice@alexv-pc)2> nodes().
['Bob@alexv-pc']
(Alice@alexv-pc)3> register(shell, self()).
true
(Alice@alexv-pc)4> receive {hello, from, Other} -> Other ! <<"whats up !">> end.
<<"whats up !">>
```
@[1-2]@[3-4]@[5-6] 
```
(Bob@alexv-pc)1> nodes().
['Alice@alexv-pc']
(Bob@alexv-pc)2> {shell, 'Alice@alexv-pc'} ! {hello, from, self()}.
{hello,from,<0.39.0>}
(Bob@alexv-pc)2> flush().
Shell got <<"whats up !">>
ok
```
@[1-2]@[3-4]@[5-6]

+++

Murphy's Law
------------

Everything that can possibly go wrong WILL go wrong

+++

But Robot system must be reliable (enough) ?!?!
-----------------------------------------------

- Specifications : WHAT the robot has to do.
- Properties : What the robot has to NOT do.
- Formal Specs : HOW the robot does it.
- Model Checking : Formal spec is respected.
- Property testing : Properties are respected
- Validation testing : Specification is respected. 
