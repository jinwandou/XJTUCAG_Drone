#ifndef UsbCamNode_H
#define UsbCamNode_H

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>

namespace usb_cam {

class UsbCamNode{

public:
    static UsbCamNode* instance();
    static void disinstance();

    bool spin();
    void publishErrNo();

public:
    UsbCamNode();
    ~UsbCamNode();

private:
  static UsbCamNode* m_pInstance;

  ros::NodeHandle nh_; //定义ROS句柄
  ros::Publisher camera_pub_; //定义ROS发布器

  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;
  UsbCam::io_method io_method;
  UsbCam::pixel_format pixel_format;

  ros::ServiceServer service_start_, service_stop_;

  bool bErrLast = false;
  bool bErrCur = true;

private:
  void init();
  void startUSBCam();
  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res );
  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res );
  bool take_and_send_image();


private:



protected:

};
}

#endif // UsbCamNode
