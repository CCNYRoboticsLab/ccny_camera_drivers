/*
 * camera_node.cpp
 *
 *  Created on: May 02, 2012
 *      Author: Carlos Jaramillo
 */

#include <ccny_opencv_cam/opencv_cam.h>

// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ccny_opencv_cam_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_opencv_cam::Camera cam(nh, nh_private);

  ros::spin();
}
// %EndTag(MAIN)%
