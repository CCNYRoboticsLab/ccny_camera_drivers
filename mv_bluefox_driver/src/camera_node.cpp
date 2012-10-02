#include <ros/ros.h>
#include <nodelet/loader.h>

#include "mv_bluefox_driver/camera.h"

int main (int argc, char **argv) {
  ros::init(argc, argv, "mv_camera");

  mv_bluefox_driver::Camera camera(ros::NodeHandle(), ros::NodeHandle("~"));

  ros::spin();
  return 0;
}

