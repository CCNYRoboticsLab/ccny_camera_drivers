#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <ccny_mvVirtualDevice/mvIMPACT_CPP/mvIMPACT_acquire.h>

//#define DEVELOP
#define PRESS_A_KEY getchar();
namespace mv_bluefox_driver
{

//-----------------------------------------------------------------------------
class ThreadParameter
//-----------------------------------------------------------------------------
{
  mvIMPACT::acquire::Device* m_pDev;
  volatile bool m_boTerminateThread;
public:
  ThreadParameter(mvIMPACT::acquire::Device* pDev) :
    m_pDev(pDev), m_boTerminateThread(false)
  {
  }
  mvIMPACT::acquire::Device* device(void) const
  {
    return m_pDev;
  }
  bool terminated(void) const
  {
    return m_boTerminateThread;
  }
  void terminateThread(void)
  {
    m_boTerminateThread = true;
    // close the camera
    m_pDev->close();
  }
};

class Camera
{
public:
  Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
  void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
  void feedImages();

  ~Camera();

private:
  ros::NodeHandle node, pnode;

  mvIMPACT::acquire::DeviceManager devMgr; // mvIMPACT Acquire device manager
  // establish access to the statistic properties
  mvIMPACT::acquire::Statistics *statistics;
  // create an interface to the device found
  mvIMPACT::acquire::FunctionInterface *fi;

  const mvIMPACT::acquire::Request* pRequest;

  ThreadParameter* threaded_device_;

  bool ok;
  boost::mutex img_buffer_mutex_lock_; ///< Thread lock on image buffer
//  unsigned char *img_frame_buffer_; ///< where data of images are stored

  int width_, height_;
  int fps, skip_frames, frames_to_skip;
  std::string device, frame;
  bool rotate;
  bool use_color_; ///< To indicate whether we want to use 3-channel (RGB) or 1-channel (grayscale) images
  bool auto_gain_; ///< To turn/on auto gain
  int expose_us_; ///< Exposure time
  std::string camera_calibration_url_;
  /* Old:
   image_transport::ImageTransport it;
   CameraInfoManager info_mgr;
   image_transport::Publisher pub;
   ros::Publisher info_pub;
   */

  image_transport::CameraPublisher camera_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_;
  boost::thread image_thread;

  pthread_t* pHandles; // pThread handles // TODO: switch these from pThreads to boost
  pthread_attr_t* pAttrs; // TODO: convert to boost
  std::vector<ThreadParameter*> threadParams;
  unsigned int devCnt;
  int lastRequestNr;
  unsigned int request_cnt;
  int request_timeout_ms_; // USB 1.1 on an embedded system needs a large timeout for the first image

  bool initMVDevices(); ///< Initializes several mv device(s). Return true if succeeded // TODO: not being used
  bool initSingleMVDevice(); ///< Initializes a single mv device. Return true if succeeded
  void setCameraSize(mvIMPACT::acquire::SettingsBlueFOX &settings, int width, int height);  ///< Sets frame size, fixing AOI for now

  bool using_pthreads; ///< indicates whether code is using the pthreaded implementation
//  static void* liveThread( void* pData );
//  static unsigned int thread_func( void* pData ); ///< The actual Thread for the device
  bool grab(sensor_msgs::ImagePtr image); ///< Returns true if frame grabbing succeeded

};

} // end namespace mv_bluefox_driver


