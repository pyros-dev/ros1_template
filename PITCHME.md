ROS1 Presentation
=================

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

C++ does NOT have "standard" tools (like python)

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
- ROS Robots : > 100 (http://wiki.ros.org/Robots)

source : http://wiki.ros.org/Metrics

+++

Permissive licensing
--------------------

- ROS core BSD license (Free to use, modify, sell)
- ROS tools: Open Source/Free licenses (free to use)



---

ROS1 Basics
===========

## Do the tutorials !

- Environment and Tools
- Packages and Dependencies
- Launchers and Parameters
- Messages, Topics and Services

+++

ROS1 Environment
----------------

`$ source setup.bash`
- modify your current SHELL (`bash`) environment.
- <span style="color:green">GOOD</span> : easy to modify quickly (`export MY_ROS_ENVVAR=42`)
- <span style="color:red">BAD</span> : hard to debug (`ps aux | grep my_ros_program`) => which environment configuration ?

+++

ROS1 Commands
-------------

`rosls`, `roscd`, `rospack`, etc.
- allows easy navigation in ROS environment
- <span style="color:green">GOOD</span> : easy to play around in your ROS environment
- <span style="color:green">GOOD</span> : allow plug-n-play for node / comm checking
- <span style="color:red">BAD</span> : not fully isolated from the rest of the system

+++

ROS1 Packages
-------------

`package.xml` and `CMakeLists.txt`
- describes a ROS package
- <span style="color:green">GOOD</span> : modular compilable source code (`catkin_make` or `catkin build`)
- <span style="color:green">GOOD</span> : modular interactive runtime (ROS API)
- <span style="color:red">BAD</span> : not easy to compose, easy to shoot yourself
- <span style="color:green">GOOD</span> : integrating with your package manager
- <span style="color:red">BAD</span> : not quite integrated, it goes into `/opt/ros/<rosdistro>`

+++

ROS1 Dependencies
-----------------

`rosdep resolve depkey`

- enable cross-platform package management
- <span style="color:green">GOOD</span> : can edit github's `ros/rosdistro` repository to add dependencies 
- <span style="color:red">BAD</span> : currently can confuse ROS packages and dependencies (BUG)

+++

ROS1 Nodes
----------

`rospy.init_node()` or `ros::init()`
- declares the current process as a ROS node
- <span style="color:green">GOOD</span> : one node match one process
- <span style="color:red">BAD</span> : special `rosmaster`, `rosout` nodes
- <span style="color:red">BAD</span> : pollute your process global state
- <span style="color:red">BAD</span> : async programming can be VERY confusing
C++ has nodelets, which means that one process has N node(let)s

+++

ROS1 Launchers
--------------

`roslaunch my_pkg my_nodes.launch`

- provides a centralised launch point for multiple process/nodes
- <span style="color:green">GOOD</span> : can load yaml files to set parameters for the system
- <span style="color:green">GOOD</span> : a launch file can include another
- <span style="color:red">BAD</span> : namespaces logic and remapping is not as simple as it could 
- <span style="color:red">BAD</span> : XML, but more programming-style logic is needed

+++

ROS1 Parameters
---------------

`$ rosparam get` or `rospy.get_param()` or `ros::NodeHandle::getParam()`

- get/set parameters for the running ROS system
- <span style="color:green">GOOD</span> : centralised structured data
- <span style="color:green">GOOD</span> : dynamically configurable 

+++

ROS1 Messages
-------------

`$ rosmsg show`

- defines message structure
- <span style="color:green">GOOD</span> : provide an API other can use
- <span style="color:green">GOOD</span> : provides serialization/deserialization crossplatform
- <span style="color:red">BAD</span> : doesnt always check and validate the data type

+++

ROS1 Topics
-----------

`rospy.Publisher()` or `ros::Publisher`

`rospy.Subscriber()` or `ros::Subscriber`

- allow ROSTCP/ROSUDP Push communication / event notifications
- <span style="color:green">GOOD</span> : low overhead (binary serialization)
- <span style="color:red">BAD</span> : lossy
- <span style="color:red">BAD</span> : Inverted control flow (deal with your callback threadS)
- <span style="color:red">BAD</span> : Cannot be used to interface with non-ROS systems
- <span style="color:green">GOOD</span> : need 2 to implement pull communication

+++

ROS1 Services
-------------

`rospy.Service()` or `ros::Service`

`rospy.ServiceProxy()` or `ros::ServiceProxy`

- allow XMLRPC Pull communication / Remote Procedure Calls
- <span style="color:red">BAD</span> : high overhead (XML)
- <span style="color:green">GOOD</span> : lossless
- <span style="color:green">GOOD</span> : Direct control flow (deal with your main thread)
- <span style="color:green">GOOD</span> : CAN be used to interface with non-ROS systems
- <span style="color:green">GOOD</span> : need 2 to implement push communication

---

Mixing C++ & Python
===================

- Different tools, community, mindset. Pick a side !
- Can mix in different ways (cffi, bindings, ...)
- ROS provide the multi-process/local-network way.
- ROS adapts its C++-like tools to python.

+++

  C++
-----

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
- Be flexible

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


+++

ROS1 Template
-------------

- A reference for ROS good practices
- "pure" C++ library
- C++ / ROS nodes and package
- ROS / python nodes and package
- "pure" python nodes and package
- Nothing beats DIY !

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
- Make a client python script
- Mutate client into a node
- Review

+++

Make a ROS package
------------------

- Follow ROS python tutorials
- Refer to [ros1_pytemplate](https://github.com/pyros-dev/ros1_template/tree/master/ros1_pytemplate) 
- Implement a Number Node
- stores a number (initialized at 1)
- provides reset() for resetting it
- provides value() for getting its value
- provides a ROS API for reset() and value()


+++

Make a Python package
---------------------

- Follow http://packaging.python.org
- Refer to [cookiecutters](https://github.com/audreyr/cookiecutter)
- Provide a python package with Accumulator module
- Accumulator stores a number N (init at 1) and a constant C
- Accumulator can do inc_mod(m) -> (N + m) mod C

+++

Turn it into a ROS package
--------------------------

- Use https://github.com/pyros-dev/catkin_pip
- Refer to [ros1_pip_pytemplate](https://github.com/pyros-dev/ros1_template/tree/master/ros1_pip_pytemplate))
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


Distributed System
==================

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
<table>
<tr><td>This</td><td>can print:</td></tr>
<tr>
<td rowspan="5">
<pre>
a=time.now()
b=time.now()
print(a>b)
</pre>
</td>
<tr><td><pre>True</pre></td></tr>
<tr><td><pre>False</pre></td></tr>
<tr><td><pre>no idea</pre></td></tr>
<tr><td><span style="color:orange"><pre>Error</pre></span></td></tr>
<tr><td><span style="color:red">Crash</span></td></tr>
</table>
              
+++

FWYTYK : no total order
-----------------------

<table>
<tr><td>This</td><td>can print:</td></tr>
<tr>
<td rowspan="7">
<pre>
int a=1
a= a+1
print(a)
</pre>
</td>
</tr>
<tr><td><pre>1</pre></td></tr>
<tr><td><pre>42</pre></td></tr>
<tr><td>...</td></tr>
<tr><td><span style="color:orange"><pre>Error</pre></span></td></tr>
<tr><td><span style="color:red">Crash</span></td></tr>
</table>
+++

Recent Research Area : CRDT - 2011
----------------------------------

- Grow-counter: we can count up ! (not down)
- Positive-Negative-counter: we can count down !
- Grow-only-set: we can group things together ! (not ungroup)
- Two-phase-set: we can ungroup things ! (only once)
- more coming...


+++


distributed programming 
-----------------------

- Nothing mainstream, except Erlang (31 years old)

```erlang
(Alice@alexv-pc)
1> net_kernel:connect_node('Bob@alexv-pc').
true
2> nodes().
['Bob@alexv-pc']
3> register(shell, self()).
true
4> receive {hello, from, Other} -> Other ! <<"whats up !">> end.
<<"whats up !">>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
(Bob@alexv-pc)
1> nodes().
['Alice@alexv-pc']
2> {shell, 'Alice@alexv-pc'} ! {hello, from, self()}.
{hello,from,<0.39.0>}
3> flush().
Shell got <<"whats up !">>
ok
```

@[10](Two Erlang VMs)
@[10,2-3](Connect a node to another)
@[10,4-5](Check connections from Alice)
@[10,12-13](Check connections from Bob)
@[10,6-7](Register current REPL in a variable)
@[10,14-15](Send message to Alice's REPL)
@[10,8-9](Receive message and pattern match)
@[10,16-18](Flush messages)
@[2,6,8,10,14](http://learnyousomeerlang.com/distribunomicon)

+++

Murphy's Law
------------

Everything that can possibly go wrong WILL go wrong
- Fail-stop failures
- Crash failures
- Omission failures
- Performance failures
- Byzantines failures


+++

But Robot must be reliable ?!?!
-------------------------------

- Specifications : WHAT the robot has to do.
- Properties : What the robot has to NOT do.
- Formal Specs : HOW the robot does it.
- Model Checking : Formal spec is respected.
- Property testing : Properties are respected
- Validation testing : Specification is respected. 
