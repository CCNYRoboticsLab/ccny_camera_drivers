/*
 * opencv_cam.cpp
 *
 *  Created on: May 02, 2012
 *      Author: Carlos Jaramillo
 */

// Simple Camera Capture (Read with set properties, such as size):
///*
#include <ccny_opencv_cam/opencv_cam.h>
#include <iostream>

namespace ccny_opencv_cam
{
Camera::Camera(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  getParams();

  controlLoop();
}

// %Tag(PARAMS)%
void Camera::getParams()
{
  loaded_params_ = false;

  nh_private_.param("show_video", show_video_, true);
  nh_private_.param("publish_as_ROS_msg", publish_as_ROS_msg_, true);
  nh_private_.param("opencv_img_topic_name", opencv_img_topic_name_, std::string("ccny_image"));


  if (!nh_private_.getParam("video_port_number", video_port_number_))
  {
    video_port_number_ = 0;
    ROS_INFO("Using video port number: %d", video_port_number_);
  }

  if (!nh_private_.getParam("camera_width", camera_width_))
  {
    camera_width_ = 320;
    ROS_INFO("Using default camera WIDTH = %d", camera_width_);
  }
  if (!nh_private_.getParam("camera_height", camera_height_))
  {
    camera_height_ = 240;
    ROS_INFO("Using default camera HEIGHT = %d", camera_height_);
  }


  loaded_params_ = true;
}
// %EndTag(PARAMS)%

void Camera::setVideoCaptureProperties(cv::VideoCapture &cap)
{
  cap.set(CV_CAP_PROP_FRAME_WIDTH, camera_width_);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera_height_);
  //  cap.set(CV_CAP_PROP_FPS, 10); // TODO: set it in Linux. You may want to look at the P/N number on the tag on the usb cord. What I know is some pro9000 are UVC compliant, and some are not.
#ifdef DEVELOP
  ROS_INFO("NEW WxH = %f x %f  at %f fps", cap.get(CV_CAP_PROP_FRAME_WIDTH),cap.get(CV_CAP_PROP_FRAME_HEIGHT), cap.get(CV_CAP_PROP_FPS));
  //  ROS_INFO("Current Capture FOURCC: %f", cap.get(CV_CAP_PROP_FOURCC));
#endif
  // NOTE: if resolution does NOT change, you may have to use the newer libv4l-based wrapper and recompile opencv2 with V4L=ON
  //  >>>> install libv4l-dev (this is how it's called in Ubuntu)
}


void Camera::controlLoop()
{
  capture_.open(video_port_number_);

  if (!capture_.isOpened()) {
      std::cout << "Failed to open video capture device " << video_port_number_ << std::endl;
  }

  setVideoCaptureProperties(capture_);

  if(show_video_)
    cv::namedWindow(CAM_WINDOW, CV_WINDOW_FREERATIO);


  image_transport::ImageTransport it(nh_private_);

  if (publish_as_ROS_msg_)
  {
    // %Tag(PUB)%
    img_pub_ = it.advertise(opencv_img_topic_name_, 1); // Publish raw image
    //        omni_img_pub_ = nh_private_.advertise<sensor_msgs::Image> (opencv_omni_topic_name_, 1);
    ROS_INFO("Publishing images via \"image_transport\"");
    // %EndTag(PUB)%
  }


  bool processing = true;
  while (processing && ros::ok())
  {
    mutex_lock_.lock();

#ifdef DEVELOP_PERFORMANCE
      double duration = static_cast<double> (cv::getTickCount());
#endif
        if (capture_.isOpened())
        {
          if (!capture_.grab())
            continue;
          capture_ >> frameCap_;

          processing = processVideo();
        }

    ros::spinOnce();

    mutex_lock_.unlock();
  }
    ROS_WARN("No longer capturing video");

  // TODO: Kill the node manually after no longer capturing

}

bool Camera::processVideo()
{

  static bool first_time = true;
  if(first_time)
  {
    channels_ = frameCap_.channels();

    if(channels_ < 3)
    {
      is_color_camera_ = false;
      color_format_encoding_ = sensor_msgs::image_encodings::MONO8;
    }
    else
    {
      is_color_camera_ = true;
      color_format_encoding_ = sensor_msgs::image_encodings::BGR8; // NOTE: CV will work with BGR encoding.
    }

    first_time = false;
  }

    cv::waitKey(CAMERA_HOLD_TIME);

    if(show_video_)
        cv::imshow(CAM_WINDOW, frameCap_);

    if (publish_as_ROS_msg_) // Publis ROS image

    {
      sensor_msgs::ImagePtr ros_img_ptr;

      cv_bridge::CvImage cv_img_frame;
      cv_img_frame.header.frame_id = "ccny_camera"; // Same timestamp and tf frame as input image
      cv_img_frame.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
      //                  cv_img_frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
      if (frameCap_.type() == CV_8UC3)
        cv_img_frame.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever

      else if (frameCap_.type() == CV_8UC1)
        cv_img_frame.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      cv_img_frame.image = frameCap_; // Your cv::Mat
      ros_img_ptr = cv_img_frame.toImageMsg();

      img_pub_.publish(ros_img_ptr);
      // It can be viewed in the command line as following:
      // $ rosrun image_view image_view image:=/sphereo_node/omni_img
    }

    return true;
}
//*/

// Sparse Matrix
/*
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
int main() {
  //  cv::Mat input = cv::Mat_<float> (cv::Size(3,3), CV_32FC1);
  cv::Mat input = (cv::Mat_<float> (3,3) << 10.0, 0, 3.2, 0, 0, 1.1, 0, 2, 0);

  std::cout << input << std::endl;

  cv::SparseMat_<float> my_sparse(input);

  cv::SparseMatIterator_<float> it = my_sparse.begin();
  cv::SparseMatIterator_<float> it_end = my_sparse.end();

  for(; it != it_end; ++it)
      {
          // take the next element from the first matrix
          float avalue = *it;
          std::cout << avalue << "\t" << std::endl;
      }

  // Note: idx is the row of the original 2-D matrix
  //       hash is the col of the original 2-D matrix
  //   => ref(idx, hash) gives value of element
  std::cout << my_sparse.ref(0,0) << std::endl;
  std::cout << my_sparse.ref(2,1) << std::endl;
}
*/
} // end namespace
