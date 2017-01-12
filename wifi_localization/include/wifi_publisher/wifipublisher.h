#ifndef PROJECT_WIFIPUBLISHER_H
#define PROJECT_WIFIPUBLISHER_H

#include <wifi_localization/WifiState.h>
#include "iwlib.h"

#include "ros/ros.h"

struct WifiDataPoint
{
  std::string ssid;
  std::string mac;
  int channel;
  int signal_strength;
  int noise;
  bool changed_signal_strength;
};

class WifiPublisher
{
public:
  WifiPublisher(ros::NodeHandle &nh, std::string wifi_interface, std::vector<float> channel_list);
  ~WifiPublisher();
  bool scan();
private:
  std::string wifi_interface_;
  int wifi_socket_;
  ros::Publisher wifi_pub_;
  std::vector<float> channel_list_;
  struct iw_scan_req scanopt_;
  wifi_localization::WifiState wifi_data_old_;
};

#endif //PROJECT_WIFIPUBLISHER_H