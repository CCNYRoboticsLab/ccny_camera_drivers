CCNY RGB-D tools 
===================================

Carlos Jaramillo  
CCNY Robotics Lab 2011  
cjaramillo@gc.cuny.edu  
http://robotics.ccny.cuny.edu/  

Overview
-----------------------------------

This stack is where we feature our custom ROS drivers that we are developing for our various projects. At the moment, we only have simple drivers for the MatrixVision BlueFox family of cameras and a (branched) driver for PointGrey cameras (tested with Chameleon monochromatic and color cameras). pgr_camera_driver is also a simple wrapper to the FlyCapture2 SDK.

This code is at an experimental stage. 

Installing
-----------------------------------

### From source ###

Create a directory where you want the package downloaded (ex. `~/ros`), 
and add it to `$ROS_PACKAGE_PATH`.

Make sure you have git installed:

    sudo apt-get install git-core

Download the stack from our repository:

    git clone https://github.com/ccny-ros-pkg/ccny_camera_drivers.git

Install any dependencies using rosdep.

    rosdep install ccny_camera_drivers

Compile the stack:

    rosmake ccny_camera_drivers

Usage
-----------------------------------

Connect your cameras and wait until they break.
