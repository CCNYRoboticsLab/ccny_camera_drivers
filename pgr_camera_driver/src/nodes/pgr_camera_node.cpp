/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
//#include <camera_calibration_parsers/parse_ini.h>
#include <camera_calibration_parsers/parse.h> // The file may be YAML or INI format
#include <std_msgs/String.h>
#include <polled_camera/publication_server.h>
#include <pgr_camera/pgr_camera.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "pgr_camera_driver/PGRCameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>
#include <fstream>
#include <sys/stat.h>

class PGRCameraNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher streaming_pub_;
  polled_camera::PublicationServer poll_srv_;
  ros::ServiceServer set_camera_info_srv_;

  // Camera
  boost::scoped_ptr < pgr_camera::Camera > cam_;
  bool running;
  int width_; ///< Camera frame width
  int height_; ///< Camera frame height

  // ROS messages
  sensor_msgs::Image img_;
  //sensor_msgs::CameraInfo cam_info_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_mgr_;
  sensor_msgs::CameraInfoPtr cam_info_ptr_;

  // Diagnostics
  int count_;
  string frame_id_;

public:
    PGRCameraNode (const ros::NodeHandle & node_handle, ros::NodeHandle param_nh):nh_ (node_handle), nh_private_ (param_nh), it_ (param_nh), cam_ (NULL), running (false),
    count_ (0)
  {
    unsigned int serial = 0;
    int serial_param;
    bool gotSerNo = nh_private_.getParam("serial_number", serial_param);

    if (! gotSerNo)
      ROS_INFO("No camera serial number provided; selecting default camera");
    else
      serial = (unsigned int) serial_param;

    // Two-stage initialization: in the constructor we open the requested camera. Most
    // parameters controlling capture are set and streaming started in configure(), the
    // callback to dynamic_reconfig.
    pgr_camera::init ();
    if (pgr_camera::numCameras () == 0)
      ROS_WARN ("Found no cameras");

    //ros::NodeHandle local_nh("~");

    // TODO: add facility to set intrinsics

    cam_.reset (new pgr_camera::Camera (serial));
    if(cam_->initCam ())
      ROS_INFO("Camera initialization succeeded!");
    else
      ROS_ERROR("Camera could not be initialized!");
  }

  void configure (pgr_camera_driver::PGRCameraConfig & config, uint32_t level)
  {
    ROS_INFO ("Reconfigure request received");

    if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
      stop ();

    loadIntrinsics (config.intrinsics_ini, config.camera_name);

    /* 
     * If Shutter and Gain are both set to auto, then auto exposure
     * will tune each one to maintain a constant exposure at each pixel. 
     *
     * If only one of Shutter/Gain is set to auto, then that value
     * will be tuned to enforce the constant exposure value at each
     * pixel, while the other value is not changed.
     *
     * If both Shutter and Gain are set to manual, then auto exposure
     * has no effect.
     *
     */

    // Exposure
    if (config.auto_exposure)
      cam_->SetExposure (true, true);
    else    
      cam_->SetExposure (false, true);


    // Shutter
    if (config.auto_shutter)
      cam_->SetShutter (true);
    else
      cam_->SetShutter (false, (float)config.shutter);
    

    // Gain
    if(config.auto_gain)
      cam_->SetGain(true);
    else
      cam_->SetGain(false, (float)config.gain);


    // video mode / framerate
    cam_->SetVideoModeAndFramerate (config.width, config.height, config.format, config.frame_rate);
    width_ = config.width;
    height_ = config.height;

    
    // TF frame
    frame_id_ = config.frame_id;

    if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
      start ();
  }

  ~PGRCameraNode ()
  {
    stop ();
    cam_.reset ();
  }

  void start ()
  {
    if (running)
      return;

    cam_->setFrameCallback (boost::bind (&PGRCameraNode::publishImage, this, _1));
    streaming_pub_ = it_.advertiseCamera ("image_raw", 1);

    cam_->start ();
    running = true;
  }

  void stop ()
  {
    if (!running)
      return;

    cam_->stop ();              // Must stop camera before streaming_pub_.
    poll_srv_.shutdown ();
    streaming_pub_.shutdown ();

    running = false;
  }

  static bool frameToImage (FlyCapture2::Image * frame, sensor_msgs::Image & image)
  {

    std::string encoding;

    // Get the raw image dimensions
    FlyCapture2::PixelFormat pixFormat;
    uint32_t rows, cols, stride;


    // NOTE: 16-bit and Yuv formats not supported
    static const char *BAYER_ENCODINGS[] = { "none", "bayer_rggb8",
      "bayer_grbg8", "bayer_gbrg8", "bayer_bggr8", "unknown"
    };
    //unsigned int bpp = frame->GetBitsPerPixel();
    FlyCapture2::BayerTileFormat bayerFmt;
    bayerFmt = frame->GetBayerTileFormat ();
    //ROS_INFO("bayer is %u", bayerFmt);
    frame->GetDimensions( &rows, &cols, &stride, &pixFormat );
    //ROS_INFO("Frame WxH = %d x %d", cols, rows); // FIXME: Dynamic resizing of camera is now working
    //ROS_INFO("Pixel Format (from Raw Image): 0x%08x", pixFormat);
    //ROS_INFO("Stride of Raw Image (The number of bytes between rows of the image): %d", stride);

    if (bayerFmt == FlyCapture2::NONE)
    {
      //ROS_INFO("Pixel Format (from Raw Image): 0x%08x", pixFormat);
      encoding = sensor_msgs::image_encodings::MONO8;
      return sensor_msgs::fillImage (image, encoding, rows,
                                     cols,
                                     stride,
                                     frame->GetData ());
    }
    else
    {
      // Create a converted image
      FlyCapture2::Image convertedImage;
      FlyCapture2::Error error;

      //encoding = BAYER_ENCODINGS[bayerFmt];
      switch ((FlyCapture2::PixelFormat) pixFormat) {
        case FlyCapture2::PIXEL_FORMAT_RAW8:
          // Convert the raw image
          error = frame->Convert(FlyCapture2::PIXEL_FORMAT_RGB, &convertedImage );
          encoding = sensor_msgs::image_encodings::RGB8; // Arbitrary encoding based on pixel format

          if (error != FlyCapture2::PGRERROR_OK)
          {
            error.PrintErrorTrace();
            return false;
          }
          // Get the raw image dimensions
          convertedImage.GetDimensions( &rows, &cols, &stride, &pixFormat );
          break;
        default:
          return false; // FIXME: Now, it's arbitrarily converting Raw images to RGB8 encoding
          // TODO: Handle encoding directly from Bayer
          break;
      } // switch
      //ROS_INFO("Stride of Converted Image (The number of bytes between rows of the image): %d", stride);

      return sensor_msgs::fillImage (image, encoding, rows,
                                     cols,
                                     stride,
                                     convertedImage.GetData ());
    } //else

  }

  bool processFrame (FlyCapture2::Image * frame, sensor_msgs::Image & img, sensor_msgs::CameraInfoPtr & cam_info)
  {
    // cam_->saveImageToFile(frame); // Just a test: comment this out if not needed

    if (!frameToImage (frame, img))
      return false;

    /// @todo Use time from frame?
    img.header.stamp = cam_info->header.stamp = ros::Time::now ();

    // Throw out any CamInfo that's not calibrated to this camera mode
    if (cam_info->K[0] != 0.0 && (img.width != cam_info->width || img.height != cam_info->height))
    {
      cam_info.reset(new sensor_msgs::CameraInfo());
    }

    // If we don't have a calibration, set the image dimensions
    if (cam_info->K[0] == 0.0)
    {
      cam_info->width = img.width = width_;
      cam_info->height = img.height = height_;
    }

    // TF frame
    img.header.frame_id = cam_info->header.frame_id = frame_id_;

    //frame->GetDimensions(&cam_info.height, &cam_info.width);
    //cam_info.width = frame->GetCols();
    //
    //              cam_info.roi.x_offset = frame->RegionX;
    //              cam_info.roi.y_offset = frame->RegionY;
    //              cam_info.roi.height = frame->Height;
    //              cam_info.roi.width = frame->Width;

    count_++;
    //ROS_INFO("count = %d", count_);
    return true;
  }


  void publishImage (FlyCapture2::Image * frame)
  {
    //sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(cam_info_->getCameraInfo()));
    // or with a member variable:
    cam_info_ptr_ = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cam_info_mgr_->getCameraInfo()));

    if (processFrame (frame, img_, cam_info_ptr_))
      streaming_pub_.publish (img_, *cam_info_ptr_);
  }

  void loadIntrinsics (string url_calib_file, string camera_name)
  {
    // Camera Information and data
    //string url_calib_file = "file://" + inifile;
    cam_info_mgr_
    = boost::shared_ptr<camera_info_manager::CameraInfoManager>(
        new camera_info_manager::CameraInfoManager(
            nh_private_,
            camera_name, // The camera name
            url_calib_file));

    if (cam_info_mgr_->isCalibrated())
       {
         ROS_INFO("PointGrey camera has loaded calibration file '%s'", url_calib_file.c_str());
       }
    else
    {
      ROS_WARN ("PointGrey Camera is not calibrated");
    }

    // NOTE: it cannot' be initialized just once...it must be refreshed on each frame.
    // WARNING: don't put this vvvvvvvv here!
    //cam_info_ptr_ = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cam_info_mgr_->getCameraInfo()));


    /*
    // Read in calibration file
    ifstream fin (inifile.c_str ());
    if (fin.is_open ())
    {
      fin.close ();
      if (
          // camera_calibration_parsers::readCalibrationIni (inifile, camera_name, cam_info_) // Reads only INI files
          camera_calibration_parsers::readCalibration (inifile, camera_name, cam_info_) // The file may be YAML or INI format.
        )
        ROS_INFO ("Loaded calibration file: '%s' for camera '%s'", inifile.c_str(), camera_name.c_str ());
      else
        ROS_WARN ("Failed to load intrinsics from camera.\n");
    }
    else
      ROS_WARN ("Intrinsics file not found: %s.", inifile.c_str ());
    */
  }

};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "pgr_camera");

  typedef dynamic_reconfigure::Server < pgr_camera_driver::PGRCameraConfig > Server;
  Server server;

  try
  {
    boost::shared_ptr < PGRCameraNode > pn (new PGRCameraNode (ros::NodeHandle(),ros::NodeHandle("~")));

    Server::CallbackType f = boost::bind (&PGRCameraNode::configure, pn, _1, _2);
    server.setCallback (f);

    ros::spin ();

  } catch (std::runtime_error & e)
  {
    ROS_FATAL ("Uncaught exception: '%s', aborting.", e.what ());
    ROS_BREAK ();
  }

  return 0;

}
