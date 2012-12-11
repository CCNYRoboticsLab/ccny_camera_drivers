MV Bluefox Driver 
===================================

Carlos Jaramillo  
CCNY Robotics Lab 2011  
cjaramillo@gc.cuny.edu  
http://robotics.ccny.cuny.edu/  

Overview
-----------------------------------

This is the unnoficial ROS driver for MatrixVision BlueFox cameras.

This code is at an experimental stage. 

Installing
-----------------------------------

### From source ###

1. Navigate to the *Meta* package dependency *ccny_mvVirtualDevice* and compile it. 

    make
          
  > NOTE: it will do several things:
    - Fetch the mvVirtualDevice tarball, 
    - Compile mvIMPACT demo apps and libraries
    - Copy libraries and include files locally
    - Copyt libraries to /usr/local/lib  (requires **root** rights)
    - Compile the *wxPropView* demo    
        
2. Test the camera with *wxPropView* just compiled for an x86 system (**ASSUMPTION**)

    $(rospack find *ccny_mvVirtualDevice*)/bin/wxPropView
    
    NOTE: if any problems, they are most-likely because the libmvLIBRARIES.so are not being found in your system!
    
Usage
-----------------------------------

Connect your cameras and wait until they break.
