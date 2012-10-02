/*
 * opencv_cam.h
 *
 *  Created on: May 02, 2012
 *      Author: Carlos Jaramillo
 */

#ifndef OPENCV_CAM_H_
#define OPENCV_CAM_H_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/SetCameraInfo.h> // Needed for setting the camera info during calibration
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h> // for publishing synchronized image and camera info topics using the standard topic naming convention
#include <camera_info_manager/camera_info_manager.h> // It provides CameraInfo, and handles SetCameraInfo service requests, saving and restoring the camera calibration data.
#include <cv_bridge/cv_bridge.h>



#define CAMERA_HOLD_TIME 100 // milliseconds

static const char CAM_WINDOW[] = "Video";

namespace ccny_opencv_cam
{
const std::string trigger_topic = "/dummy"; ///< Name of topic to subscribe to


class Camera {
public:
  Camera(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~Camera()
  {
  };


private:

  void getParams();
  void controlLoop();
  bool processVideo();
  void setVideoCaptureProperties(cv::VideoCapture &cap);


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  cv::Mat frameCap_;
  cv::VideoCapture capture_; ///< Create an object that decodes the input video stream.

  image_transport::Publisher img_pub_;
  std::string opencv_img_topic_name_;

  boost::mutex mutex_lock_; ///< Thread lock on subscribed input images

  bool show_video_; ///< to show OpenCV window
  bool publish_as_ROS_msg_; ///< to publish as ROS image message
  bool loaded_params_;
  int video_port_number_;
  int camera_width_;
  int camera_height_;
  int channels_; ///< Frame channels
  std::string color_format_encoding_;  /// Color encoding from sensor_msgs::image_encodings
  bool is_color_camera_; ///< Determines if camera is in RGB mode or mono


};

} // end namespace

#endif /* OPENCV_CAM_H_ */
