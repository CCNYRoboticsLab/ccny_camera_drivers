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

#include "pgr_camera/pgr_camera.h"

// The following macro makes the source easier to read as it's not 66% error handling!
#define PGRERROR_OK FlyCapture2::PGRERROR_OK
#define PRINT_ERROR_AND_RETURN_FALSE {ROS_ERROR(error.GetDescription()); return false;}
#define PRINT_ERROR {ROS_ERROR(error.GetDescription());}

namespace pgr_camera
{

static unsigned int cameraNum = 0;


void PrintFormat7Capabilities( FlyCapture2::Format7Info fmt7Info )
{
    ROS_INFO(
        "Max image pixels: (%u, %u)\n"
        "Image Unit size: (%u, %u)\n"
        "Offset Unit size: (%u, %u)\n"
        "Pixel format bitfield: 0x%08x\n",
        fmt7Info.maxWidth,
        fmt7Info.maxHeight,
        fmt7Info.imageHStepSize,
        fmt7Info.imageVStepSize,
        fmt7Info.offsetHStepSize,
        fmt7Info.offsetVStepSize,
        fmt7Info.pixelFormatBitField );
}

void PrintError( FlyCapture2::Error error )
{
    error.PrintErrorTrace();
}

void init()
{
  FlyCapture2::Error error;
  FlyCapture2::BusManager busMgr;
  for (int tries = 0; tries < 5; ++tries)
  {
    if ((error = busMgr.GetNumOfCameras(&cameraNum)) != PGRERROR_OK)
      ROS_ERROR (error.GetDescription ());
    if (cameraNum)
    {
      ROS_INFO ("Found %d cameras", cameraNum);
      unsigned int serNo;
      for (unsigned int i = 0; i < cameraNum; i++)
      {
        if ((error = busMgr.GetCameraSerialNumberFromIndex(i, &serNo)) != PGRERROR_OK)
          ROS_ERROR (error.GetDescription ());
        else
          ROS_INFO ("Camera %u: S/N %u", i, serNo);
      }
    }
    if (error == PGRERROR_OK)
      return;
    usleep(200000);
  }
}

// FIXME: How to make this a member function?
void frameDone(FlyCapture2::Image * frame, const void *pCallbackData)
{
  // ROS_INFO("Pixel Format: 0x%08x", frame->GetPixelFormat());
  Camera *camPtr = (Camera *)pCallbackData;
  if (!camPtr->userCallback_.empty())
  {
    // TODO: thread safety OK here?
    boost::lock_guard<boost::mutex> guard(camPtr->frameMutex_);
    camPtr->userCallback_(frame);
    //ROS_INFO("in frameDone");
  }
  else
  {
    ROS_WARN ("User callback empty!");
  }
}

size_t numCameras()
{
  return cameraNum;
}

Camera::Camera()
{
  Camera(0);
}

Camera::Camera(unsigned int serNo) :
  camIndex(0), camSerNo(serNo), frameRate_(FlyCapture2::FRAMERATE_30)
{
  setup();
}

bool Camera::saveImageToFile(FlyCapture2::Image * rawImage)
{
  static int imageCount=0;
  FlyCapture2::Error error;
  FlyCapture2::CameraInfo camInfo;
  error = camPGR_.GetCameraInfo(&camInfo);


  // Get the raw image dimensions
  FlyCapture2::PixelFormat pixFormat;
  unsigned int rows, cols, stride;
  rawImage->GetDimensions( &rows, &cols, &stride, &pixFormat );

  // Create a converted image
  FlyCapture2::Image convertedImage;

  // Convert the raw image
  error = rawImage->Convert(FlyCapture2::PIXEL_FORMAT_BGRU, &convertedImage );
  if (error != PGRERROR_OK)
  {
    PrintError( error );
    return false;
  }
  // Create a unique filename
  char filename[512];
  sprintf( filename, "/tmp/%u-%d.bmp", camInfo.serialNumber, imageCount );
  ROS_INFO("Filename: %s", filename);

  // Save the image. If a file format is not passed in, then the file
  // extension is parsed to attempt to determine the file format.
  error = convertedImage.Save( filename );
  if (error != PGRERROR_OK)
  {
    PrintError( error );
  }

  imageCount++;

  return true;
}

bool Camera::setup()
{
  ///////////////////////////////////////////////////////
  // Get a camera:
  FlyCapture2::Error error;
  FlyCapture2::BusManager busMgr;
  FlyCapture2::PGRGuid guid;
  unsigned int N;
  if ((error = busMgr.GetNumOfCameras(&N)) != PGRERROR_OK)
    ROS_ERROR (error.GetDescription ());
  if (camSerNo == 0)
  {
    if ((error = busMgr.GetCameraFromIndex(camIndex, &guid)) != PGRERROR_OK)
      PRINT_ERROR_AND_RETURN_FALSE;
    ROS_INFO ("did busMgr.GetCameraFromIndex(0, &guid)");
  }
  else
  {
    ROS_INFO("Creating camera w/ serial: %u", camSerNo);
    if ((error = busMgr.GetCameraFromSerialNumber(camSerNo, &guid)) != PGRERROR_OK)
      PRINT_ERROR_AND_RETURN_FALSE;
  }

  ROS_INFO ("Setup successful");
  return true;

}

void Camera::setFrameCallback(boost::function<void(FlyCapture2::Image *)> callback)
{
  userCallback_ = callback;
}

//typedef void (Camera::*funcPtr)(Image*, void*);

bool Camera::initCam()
{
  // Start capturing images:
  FlyCapture2::Error error;
  FlyCapture2::BusManager busMgr;
  FlyCapture2::PGRGuid guid;

  if (camSerNo == 0) //Get first camera by default
  {
    if ((error = busMgr.GetCameraFromIndex(0, &guid)) != PGRERROR_OK)
      PRINT_ERROR;
  }
  else
  {
    if ((error = busMgr.GetCameraFromSerialNumber(camSerNo, &guid)) != PGRERROR_OK)
      PRINT_ERROR;
  }

  if ((error = camPGR_.Connect(&guid)) != PGRERROR_OK)
    PRINT_ERROR;
  ROS_INFO ("Camera GUID = %u %u %u %u", guid.value[0], guid.value[1], guid.value[2], guid.value[3]);

  // Set to software triggering:
  FlyCapture2::TriggerMode triggerMode;
  if ((error = camPGR_.GetTriggerMode(&triggerMode)) != PGRERROR_OK)
    PRINT_ERROR;

  // Set camera to trigger mode 0
  triggerMode.onOff = false;

  if ((error = camPGR_.SetTriggerMode(&triggerMode)) != PGRERROR_OK)
    PRINT_ERROR;

  // Set other camera configuration stuff:
  FlyCapture2::FC2Config fc2Config;
  if ((error = camPGR_.GetConfiguration(&fc2Config)) != PGRERROR_OK)
    PRINT_ERROR;
  fc2Config.grabMode = FlyCapture2::DROP_FRAMES; // supposedly the default, but just in case..
  if ((error = camPGR_.SetConfiguration(&fc2Config)) != PGRERROR_OK)
    PRINT_ERROR;
  ROS_INFO ("Setting video mode to VIDEOMODE_640x480Y8, framerate to FRAMERATE_30...");
  if ((error = camPGR_.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_30))
      != PGRERROR_OK)
    ROS_ERROR (error.GetDescription ());
  else
    ROS_INFO ("...success");
  FlyCapture2::EmbeddedImageInfo embedInfo;
  embedInfo.frameCounter.onOff = true;
  if ((error = camPGR_.SetEmbeddedImageInfo(&embedInfo)) != PGRERROR_OK)
    PRINT_ERROR;

  FlyCapture2::CameraInfo camInfo;
  if ((error = camPGR_.GetCameraInfo(&camInfo)) != PGRERROR_OK)
    PRINT_ERROR;
  ROS_INFO ("camInfo.driverName = %s", camInfo.driverName);
  ROS_INFO ("camInfo.firmwareVersion = %s", camInfo.firmwareVersion);
  ROS_INFO ("camInfo.isColorCamera = %d", camInfo.isColorCamera);

  // FIXME: breaks dynamic Resizing of image
  // Query for available Format 7 modes
  FlyCapture2::Format7Info fmt7Info;
  bool supported;

  // TODO: Setup modes here (working as Raw for now...either Mono or Color...whatever the camera's raw is)
//  if(camInfo.isColorCamera)
//  {
//    k_fmt7Mode_ = FlyCapture2::MODE_0; // TODO: make dynamic
//    k_fmt7PixFmt_ = FlyCapture2::PIXEL_FORMAT_BGR; // TODO: make dynamic
//  }
//  else // Mono
//  {
//    k_fmt7Mode_ = FlyCapture2::MODE_1; // TODO: make dynamic
//    k_fmt7PixFmt_ = FlyCapture2::PIXEL_FORMAT_MONO8; // TODO: make dynamic
        k_fmt7Mode_ = FlyCapture2::MODE_0; // TODO: make dynamic
        k_fmt7PixFmt_ = FlyCapture2::PIXEL_FORMAT_RAW8; // TODO: make dynamic
//  }
  fmt7Info.mode = k_fmt7Mode_;
  error = camPGR_.GetFormat7Info( &fmt7Info, &supported );
  if (error != PGRERROR_OK)
  {
    PrintError( error );
    return false;
  }
  PrintFormat7Capabilities( fmt7Info );
  FlyCapture2::Format7ImageSettings fmt7ImageSettings;
  fmt7ImageSettings.mode = k_fmt7Mode_;
  fmt7ImageSettings.offsetX = 0;
  fmt7ImageSettings.offsetY = 0;
  fmt7ImageSettings.width = fmt7Info.maxWidth;
  fmt7ImageSettings.height = fmt7Info.maxHeight;
  fmt7ImageSettings.pixelFormat = k_fmt7PixFmt_;

  ROS_INFO("Pixel Format: 0x%08x", k_fmt7PixFmt_);


  bool valid;
  FlyCapture2::Format7PacketInfo fmt7PacketInfo;

  // Validate the settings to make sure that they are valid
  error = camPGR_.ValidateFormat7Settings(
      &fmt7ImageSettings,
      &valid,
      &fmt7PacketInfo );
  if (error != PGRERROR_OK)
  {
    PrintError( error );
    return false;
  }

  if ( !valid )
  {
    // Settings are not valid
    printf("Format7 settings are not valid\n");
    return false;
  }

  // Set the settings to the camera
  error = camPGR_.SetFormat7Configuration(
      &fmt7ImageSettings,
      fmt7PacketInfo.recommendedBytesPerPacket );
  if (error != PGRERROR_OK)
  {
    PrintError( error );
    return false;
  }
  return true;
}

void Camera::start()
{
  FlyCapture2::Error error;
  if (camPGR_.IsConnected())
  {
    ROS_INFO ("IsConnected returned true");
  }
  else
    ROS_INFO ("IsConnected returned false");

  if ((error = camPGR_.StartCapture(frameDone, (void *) this)) != PGRERROR_OK)
  { //frameDone, (void*) this)) != PGRERROR_OK) {
    ROS_ERROR (error.GetDescription ());
  }
  else
  {
    ROS_INFO ("StartCapture succeeded.");
  }

}

void Camera::stop()
{
  FlyCapture2::Error error;
  if ((error = camPGR_.StopCapture()) != PGRERROR_OK)
    PRINT_ERROR;
}

void Camera::SetVideoModeAndFramerate(unsigned int width, unsigned int height, string format, double rate)
{
  // TODO: support fractional frame rates
  // TODO: support more types of color cameras (just getting RAW data and being handled by the ROS image topic)
  //      It ignores bayasian encoding for now (for color cameras). It forces conversion from raw to RGB8 pixel format encoding.
  // TODO: support additional modes
  using namespace FlyCapture2;
  bool unknown = false;
  VideoMode vidMode;
  FrameRate frameRate;
  ROS_INFO_STREAM("Requested Width: " << width << " Height: " << height << " Format: '" << format <<"'");
  switch (width)
  {
    case 640:
      ROS_INFO("Height: %d",height);
      switch (height)
      {
        case 480:
          if (format == "Y8")
            vidMode = VIDEOMODE_640x480Y8;
          else if (format == "Y16")
            vidMode = VIDEOMODE_640x480Y16;
          else if (format == "RGB")
            vidMode = VIDEOMODE_640x480RGB;
          else {
            unknown = true;
            }
          break;
        default:
          unknown = true;
          break;
      }
      break;
    case 1280:
      switch (height)
      {
        case 960:
          if (format == "Y8")
          {
            vidMode = VIDEOMODE_1280x960Y8;
          }
          else if (format == "Y16")
          {
            vidMode = VIDEOMODE_1280x960Y16;
          }
          else if (format == "RGB")
          {
            vidMode = VIDEOMODE_1280x960RGB;
          }
          else {
            unknown = true;
          }
          break;
        default:
          unknown = true;
          break;
      }
      break;
    default:
      break;
  }

  if (unknown)
  {
    ROS_ERROR ("Unknown/unsupported video mode - mode not set");
    return;
  }

  unknown = false;
  // The following hardcoded numbers are from testing with
  // a FireflyMV USB (mono) camera using FlyCap2's configuration GUI
  // TODO: determine whether they are camera specific and if so, so this a better way..
  if (format == "FORMAT7")
  {
    frameRate = FRAMERATE_FORMAT7;
  }
  else if (rate >= 1.151 && rate < 7.606)
  {
    frameRate = FRAMERATE_7_5;
  }
  else if (rate >= 7.606 && rate < 15.211)
  {
    frameRate = FRAMERATE_15;
  }
  else if (rate >= 15.211 && rate < 30.430)
  {
    frameRate = FRAMERATE_30;
  }
  else if (rate >= 30.430 && rate < 60.861)
  {
    frameRate = FRAMERATE_60;
  }
  else
  {
    ROS_ERROR ("Unsupported frame rate");
    return;
  }

  Error error;
  ROS_INFO ("Attempting to set mode for width = %u height = %u format = %s frame_rate = %f",
      width, height, format.c_str (), rate);
  if ((error = camPGR_.SetVideoModeAndFrameRate(vidMode, frameRate)) != PGRERROR_OK)
  {
    ROS_ERROR (error.GetDescription ());
    ROS_ERROR ("Video mode and frame rate not set");
    ROS_ERROR ("vidMode = %u", vidMode);
    ROS_ERROR ("frameRate = %u", frameRate);
    return;
  }
  ROS_INFO ("Video mode set to %u, and frame rate set to %u", vidMode, frameRate);

  FlyCapture2::Property prop;
  prop.type = FRAME_RATE;
  prop.autoManualMode = false;
  prop.onOff = true;
  prop.absControl = true;
  prop.absValue = rate;
  if ((error = camPGR_.SetProperty(&prop)) != PGRERROR_OK)
  {
    ROS_ERROR (error.GetDescription ());
  }

}

void Camera::SetExposure(bool _auto, bool onoff, unsigned int value)
{
  FlyCapture2::Property prop;
  FlyCapture2::Error error;
  prop.type = FlyCapture2::AUTO_EXPOSURE;
  prop.autoManualMode = _auto;
  prop.onOff = onoff;
  prop.valueA = value;
  if ((error = camPGR_.SetProperty(&prop)) != PGRERROR_OK)
  {
    ROS_ERROR (error.GetDescription ());
  }
}

void Camera::SetGain(bool _auto, float value)
{
  FlyCapture2::Property prop;
  FlyCapture2::Error error;
  prop.type = FlyCapture2::GAIN;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  prop.absValue = value;
  if ((error = camPGR_.SetProperty(&prop)) != PGRERROR_OK)
  {
    ROS_ERROR (error.GetDescription ());
  }

  return;
}


void Camera::SetShutter (bool _auto, float value)
{
  FlyCapture2::Property prop;
  FlyCapture2::Error error;
  prop.type = FlyCapture2::AUTO_EXPOSURE;
  prop.autoManualMode = _auto;
  prop.onOff = true;
  prop.absValue = value;
  if ((error = camPGR_.SetProperty(&prop)) != PGRERROR_OK)
  {
    ROS_ERROR (error.GetDescription ());
  }

  return;
}

} // namespace pgrcamera
