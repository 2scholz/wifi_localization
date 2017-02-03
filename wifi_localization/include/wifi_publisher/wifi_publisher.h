#ifndef PROJECT_WIFIPUBLISHER_H
#define PROJECT_WIFIPUBLISHER_H

#include <wifi_localization/WifiState.h>
#include "ros/ros.h"
#include <wifi_publisher/wifi_scan.h>

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
  WifiPublisher(ros::NodeHandle &nh, std::string wifi_interface, std::vector<unsigned int> channel_list, int ms_till_discard, int chans_per_scan);
  ~WifiPublisher();
  bool scan();
  void reset();
private:
  unsigned int channel_to_freq(unsigned int channel);
  int freq_to_channel(unsigned int channel);
  std::string wifi_interface_;
  ros::Publisher wifi_pub_;
  wifi_localization::WifiState wifi_data_old_;
  struct netlink_channel* channel_;
  struct bss_infos* scan_data_;
  uint32_t frequency_list_[40];
  int ms_till_discard_;
  int chans_per_scan_;
  int current_frequency_;
  int sequential_failures_;
  std::map<std::string, int> previous_signal_strength_;
};

#endif //PROJECT_WIFIPUBLISHER_H