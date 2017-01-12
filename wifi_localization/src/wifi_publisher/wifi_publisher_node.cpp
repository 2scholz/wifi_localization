#include <ros/ros.h>
#include "wifi_localization/WifiState.h"
#include <wifi_publisher/wifipublisher.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_publisher_test");
  ros::NodeHandle nh;
  std::string wifi_interface = "wlan0";
  std::vector<float> channel_list_2GHz = {1,2,3,4,5,6,7,8,9,10,11,12,13};
  std::vector<float> channel_list_5GHz = {36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140};
  std::vector<float> channel_list = channel_list_2GHz;
  channel_list.insert(channel_list.end(),channel_list_5GHz.begin(), channel_list_5GHz.end());
  nh.param("/wifi_publisher_test/wifi_interface", wifi_interface, wifi_interface);
  nh.param("/wifi_publisher_test/channel_list", channel_list, channel_list);


  WifiPublisher publisher(nh, wifi_interface, channel_list_2GHz);
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    publisher.scan();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
