#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"

MYNTEYE_USE_NAMESPACE

namespace enc = sensor_msgs::image_encodings;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::NodeHandle private_nh_("~");

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  std::string serial_num; 
  if (private_nh_.getParam("serial_number", serial_num))
  {
    ROS_INFO("Got param serial_number = %s", serial_num.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param 'serial_number'");
    return 1;
  }
  
  Context context;
  auto &&devices = context.devices();
  size_t num_devices = devices.size();
  int device_found = -1;
  for (int i = 0; i < num_devices; i++) {
    auto &&device = devices[i];
    auto &&name = device->GetInfo(Info::DEVICE_NAME);
    auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
    ROS_INFO("Found device with serial number %s", serial_number.c_str());
    if (serial_num == serial_number) {
      ROS_INFO("  - Match!");
      device_found = i;
    }
  }
  
  if (device_found < 0) {
    ROS_ERROR("Device with given serial number was not found.");
    return 1;
  }
  std::shared_ptr<API> &&api = API::Create(argc, argv, device_found);
  
  if (api != nullptr) {
    std::cout << "Successfully opened device with serial number " << api->GetInfo()->serial_number << std::endl;
  }
  else {
    return 1;
  }
  
  api->EnableMotionDatas();
  api->EnableStreamData(Stream::LEFT_RECTIFIED);
  api->EnableStreamData(Stream::RIGHT_RECTIFIED);
  api->Start(Source::ALL);
  
  image_transport::ImageTransport it_mynteye(n);
  image_transport::Publisher pub_left_rect = it_mynteye.advertise("left_rect", 1);
  image_transport::Publisher pub_right_rect = it_mynteye.advertise("right_rect", 1);
  
  ros::Rate loop_rate(15);
  
  cv::namedWindow("frame");

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    api->WaitForStreams();
    auto &&left_data = api->GetStreamData(Stream::LEFT_RECTIFIED);
    auto &&right_data = api->GetStreamData(Stream::RIGHT_RECTIFIED);

    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      //cv::Mat img;
      //cv::hconcat(left_data.frame, right_data.frame, img);
      //cv::imshow("frame", img);
      ros::Time now = ros::Time::now();
      
      auto left_header = std_msgs::Header();
      left_header.stamp = now;
      sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(left_header, enc::MONO8, left_data.frame).toImageMsg();
      pub_left_rect.publish(left_msg);
      
      auto right_header = std_msgs::Header();
      right_header.stamp = now;
      sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(right_header, enc::MONO8, right_data.frame).toImageMsg();
      pub_right_rect.publish(right_msg);
    }
    
    /*char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }*/

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
