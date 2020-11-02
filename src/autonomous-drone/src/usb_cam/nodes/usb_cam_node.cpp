#include <usb_cam/usb_cam_node.h>

namespace usb_cam {
/////////////////////////////////////////////////////
/// \brief UsbCamNode::UsbCamNode
/// \param parent
/////////////////////////////////////////////////////
UsbCamNode* UsbCamNode::m_pInstance = NULL;

UsbCamNode *UsbCamNode::instance()
{
    if (m_pInstance == NULL)
    {
        ROS_INFO("Starting UsbCamNode instance");
        m_pInstance = new UsbCamNode();
    }
    return m_pInstance;
}

void UsbCamNode::disinstance()
{
    if (m_pInstance != NULL)
    {
        ROS_INFO("Starting ~UsbCamNode ");
        delete m_pInstance;
        m_pInstance = NULL;
    }
}

UsbCamNode::UsbCamNode() :
    node_("~")
{
  ROS_INFO("Starting UsbCamNode ");
  init();
}

UsbCamNode::~UsbCamNode()
{
  cam_.shutdown();
}

void UsbCamNode::init()
{
  // advertise the main image topic
  image_transport::ImageTransport it(node_);
  image_pub_ = it.advertiseCamera("image_raw", 1);



  // grab the parameters
  node_.param("video_device", video_device_name_, std::string("/dev/video0"));
  node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
  node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
  node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
  node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
  // possible values: mmap, read, userptr
  node_.param("io_method", io_method_name_, std::string("mmap"));
  node_.param("image_width", image_width_, 640);
  node_.param("image_height", image_height_, 480);
  node_.param("framerate", framerate_, 30);
  // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
  node_.param("pixel_format", pixel_format_name_, std::string("yuyv"));
  // enable/disable autofocus
  node_.param("autofocus", autofocus_, false);
  node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
  // enable/disable autoexposure
  node_.param("autoexposure", autoexposure_, true);
  node_.param("exposure", exposure_, 100);
  node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
  // enable/disable auto white balance temperature
  node_.param("auto_white_balance", auto_white_balance_, true);
  node_.param("white_balance", white_balance_, 4000);

  // load the camera info
  node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
  node_.param("camera_name", camera_name_, std::string("head_camera"));
  node_.param("camera_info_url", camera_info_url_, std::string(""));
  cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

  // create Services
  service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
  service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);
  camera_pub_ = nh_.advertise<std_msgs::Int16>("elisa_driver", 1); //定义发布器

  // check for default camera info
  if (!cinfo_->isCalibrated())
  {
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = img_.header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
  }


  ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
      image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

  // set the IO method
  io_method = UsbCam::io_method_from_string(io_method_name_);
  if(io_method == UsbCam::IO_METHOD_UNKNOWN)
  {
    ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
    node_.shutdown();
    return;
  }

  // set the pixel format
   pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
  {
    ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());

    node_.shutdown();
    return;
  }

  startUSBCam();
}

void UsbCamNode::startUSBCam()
{
  // start the camera
  cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
       image_height_, framerate_);

  // set camera parameters
  if (brightness_ >= 0)
  {
    cam_.set_v4l_parameter("brightness", brightness_);
  }

  if (contrast_ >= 0)
  {
    cam_.set_v4l_parameter("contrast", contrast_);
  }

  if (saturation_ >= 0)
  {
    cam_.set_v4l_parameter("saturation", saturation_);
  }

  if (sharpness_ >= 0)
  {
    cam_.set_v4l_parameter("sharpness", sharpness_);
  }

  if (gain_ >= 0)
  {
    cam_.set_v4l_parameter("gain", gain_);
  }

  // check auto white balance
  if (auto_white_balance_)
  {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
  }
  else
  {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
    cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
  }

  // check auto exposure
  if (!autoexposure_)
  {
    // turn down exposure control (from max of 3)
    cam_.set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    cam_.set_v4l_parameter("exposure_absolute", exposure_);
  }

  // check auto focus
  if (autofocus_)
  {
    cam_.set_auto_focus(1);
    cam_.set_v4l_parameter("focus_auto", 1);
  }
  else
  {
    cam_.set_v4l_parameter("focus_auto", 0);
    if (focus_ >= 0)
    {
      cam_.set_v4l_parameter("focus_absolute", focus_);
    }
  }

  std_msgs::Int16 result; //create a variable of type "Int16",
  result.data = cam_.getErrNo();
  camera_pub_.publish(result); // publish the value--of type Int16--
}

bool UsbCamNode::service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  cam_.start_capturing();
  return true;
}

bool UsbCamNode::service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  cam_.stop_capturing();
  return true;
}

bool UsbCamNode::take_and_send_image()
{    
  // grab the image
  cam_.grab_image(&img_);

  // grab the camera info
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
  ci->header.frame_id = img_.header.frame_id;
  ci->header.stamp = img_.header.stamp;

  // publish the image
  image_pub_.publish(img_, *ci);

  return true;
}

bool UsbCamNode::spin()
{
  ros::Rate loop_rate(this->framerate_);
  while (node_.ok())
  {
    if (cam_.is_capturing()) {

      if (!take_and_send_image())
        ROS_WARN("USB camera did not respond in time.");
    }
    publishErrNo();
    ros::spinOnce();
    loop_rate.sleep();

  }
  return true;
}

void UsbCamNode::publishErrNo()
{
  std_msgs::Int16 result; //create a variable of type "Int16",
  result.data = cam_.getErrNo();
  if (result.data != 0 ) {
    cam_.reset();
    // start the camera
    cam_.shutdown();
    startUSBCam();
    bErrCur = true;
  }
  else
  {
      bErrCur = false;
  }

  if(bErrLast != bErrCur)
  {
//      ROS_INFO("=============send cameraDriver value is: %d===========",result.data);
//      camera_pub_.publish(result); // publish the value--of type Int16--
  }

  bErrLast = bErrCur;


}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrx_t4_usb_cam");
  usb_cam::UsbCamNode::instance()->spin();
  return EXIT_SUCCESS;
}

