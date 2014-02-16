ros-aitown
==========

Integrates [aitown](http://tnick.github.io/aitown/) 
components with [ROS](http://www.ros.org/).


Packages
========

Software in [ROS](http://www.ros.org/) is distributed
as [packages](http://wiki.ros.org/Packages).

Following packages are provided:

ros\_aitown\_dstorage
---------------------

A node providing global id-management. This exposes
aitown-dstorage library.


Building
========

Create a new directory, say ros-aitown, and clone the
repository inside `src` directory:

```Bash
mkdir ~/ros-aitown
cd ~/ros-aitown
git clone https://github.com/TNick/ros-aitown.git src
```

Next, initialise a ros workspace and build it:

```Bash
cd src
catkin_init_workspace
cd ..
catkin_make
```

If `catkin_init_workspace` does not work take a look
at 
[Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

