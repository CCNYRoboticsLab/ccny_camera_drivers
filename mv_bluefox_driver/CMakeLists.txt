cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# boost
rosbuild_add_boost_directories()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# TODO: add dynamic reconfiguration
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

# set(MVIMPACT_LIBRARIES   mvDeviceManager mvBlueFOX??? dl??? )
set(MVIMPACT_LIBRARIES   mvDeviceManager) # The mvDeviceManager library (library is external, and should be able to be found in the system path, such as /usr/local/lib/ )

################################################################
# Build libraries
################################################################

rosbuild_add_library(mv_bluefox_driver_nodelet src/nodelets.cpp src/camera.cpp)

################################################################
# Build mv_bluefox_driver (Node and nodelet)
################################################################

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# link the nodelet to the mvDeviceManager library (library is external, and should be able to be found in the system path, such as /usr/local/lib/ )
target_link_libraries(mv_bluefox_driver_nodelet
         ${MVIMPACT_LIBRARIES}
         boost_signals 
         boost_system
         ) 
 
rosbuild_add_executable(camera_node src/camera_node.cpp src/camera.cpp)
target_link_libraries(camera_node
         ${MVIMPACT_LIBRARIES}
         boost_signals 
         boost_system
         )