#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "mv_bluefox_driver/camera.h"


namespace mv_bluefox_driver
{

class CameraNodelet : public nodelet::Nodelet
{
public:
  CameraNodelet()
  {
  }

  void onInit()
  {
    ros::NodeHandle node = getNodeHandle();
    ros::NodeHandle pnode = getPrivateNodeHandle();

    camera = new Camera(node, pnode);
  }

  ~CameraNodelet()
  {
    if (camera)
      delete camera;
  }

private:
  mv_bluefox_driver::Camera *camera;
};

} // end namespace mv_bluefox_driver

//typedef mv_bluefox_driver::CameraNodelet CameraNodelet;
PLUGINLIB_DECLARE_CLASS(mv_bluefox_driver, CameraNodelet, mv_bluefox_driver::CameraNodelet, nodelet::Nodelet);

