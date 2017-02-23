#include <ros/ros.h>
#include "wifi_localization/WifiState.h"
#include <wifi_publisher/wifi_publisher.h>
#include <wifi_publisher/wifi_scan.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_publisher_test");
  ros::NodeHandle nh;
  std::string wifi_interface = "wlan0";

  // How many channels should be scanned per scan? More channels means longer waiting time until results are returned.
  int chans_per_scan = -1;

  // Results that are older than this threshold are discared.
  int ms_till_discard = -1;

  // How many scans per second should be done at most?
  double scans_per_sec = 1.0;

  // If no channels were given by the user, the entire spectrum is scanned.
  std::vector<int> channel_list_2GHz = {1,2,3,4,5,6,7,8,9,10,11,12,13};
  std::vector<int> channel_list_5GHz = {36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140};
  std::vector<int> channel_list = channel_list_2GHz;
  channel_list.insert(channel_list.end(),channel_list_5GHz.begin(), channel_list_5GHz.end());
  channel_list = {1,6,11,36,52,64,100,108,136};

  nh.param("/wifi_publisher/wifi_interface", wifi_interface, wifi_interface);
  nh.param("/wifi_publisher/channel_list", channel_list, channel_list);
  nh.param("/wifi_publisher/chans_per_scan", chans_per_scan, chans_per_scan);
  nh.param("wifi_publisher/ms_till_discard", ms_till_discard, ms_till_discard);
  nh.param("/wifi_publisher/scans_per_sec", scans_per_sec, scans_per_sec);

  std::vector<unsigned int> channel_list_unsigned (channel_list.begin(), channel_list.end());



  //WifiPublisher publisher(nh, wifi_interface, channel_list_2GHz);

  WifiPublisher publisher(nh,wifi_interface,channel_list_unsigned,ms_till_discard, chans_per_scan);
  ros::Rate loop_rate(scans_per_sec);

  while(ros::ok())
  {
    publisher.scan();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
