#include <wifi_publisher/wifi_publisher.h>
#include "wifi_localization/WifiState.h"

WifiPublisher::WifiPublisher(ros::NodeHandle &nh, std::string wifi_interface, std::vector<unsigned int> channel_list, int ms_till_discard, int chans_per_scan):wifi_interface_(wifi_interface), ms_till_discard_(ms_till_discard), chans_per_scan_(chans_per_scan), current_frequency_(0), sequential_failures_(0)
{
  channel_ = new struct netlink_channel;

  wifi_scan_init(wifi_interface.c_str(), channel_);
  wifi_pub_ = nh.advertise<wifi_localization::WifiState>("wifi_data", 1);
  scan_data_ = new struct bss_infos;
  scan_data_->scan_data_length = 200;
  scan_data_->sequence = 0;
  scan_data_->data = new bss_info[200];
  int i = 0;
  for(auto &e:channel_list)
  {
    frequency_list_[i] = channel_to_freq(e);
    i++;
  }
  frequency_list_[i] = 0;

}


WifiPublisher::~WifiPublisher()
{
  wifi_scan_close(channel_);
  delete channel_;
  delete[] scan_data_;
}

bool WifiPublisher::scan()
{
  int failure;
  if(chans_per_scan_ != -1)
  {
    uint32_t temp_frequency_list[chans_per_scan_+1];
    for(int i = 0; i<chans_per_scan_;i++)
    {
      temp_frequency_list[i] = frequency_list_[current_frequency_];
      current_frequency_++;
      if(frequency_list_[current_frequency_] == 0)
      {
        current_frequency_ = 0;
      }
    }
    failure = wifi_scan_channels(channel_,temp_frequency_list, scan_data_);
  }
  else
  {
    failure = wifi_scan_channels(channel_,frequency_list_,scan_data_);
  }
  if(failure == -1)
  {
    ++sequential_failures_;
    if(sequential_failures_>20)
    {
      ROS_INFO("WiFi scan failed. ");
    }
    reset();
    return false;
  }
  else if(failure == 0)
  {
    sequential_failures_ = 0;
  }

  wifi_localization::WifiState wifi_data;

  wifi_data.header.stamp = ros::Time::now();

  WifiDataPoint wifi_data_point;
  for(int i=0;i<scan_data_->sequence;i++)
  {
    if((scan_data_->data[i].seen_ms_ago<ms_till_discard_) || (ms_till_discard_ == -1))
    {
      int signal_strength = int(scan_data_->data[i].signal_mbm)/100;
      std::string mac = std::string(scan_data_->data[i].bssid);

      wifi_data.ssids.push_back(std::string(scan_data_->data[i].ssid));
      wifi_data.macs.push_back(mac);
      wifi_data.channels.push_back(freq_to_channel(scan_data_->data[i].frequency));
      wifi_data.strengths.push_back(signal_strength);
      wifi_data.ms.push_back(scan_data_->data[i].seen_ms_ago);
      wifi_data.changed_signal_strength.push_back(true);

      auto it = previous_signal_strength_.find(mac);

      if(it != previous_signal_strength_.end())
        if(it->second != signal_strength)
          wifi_data.changed_signal_strength.back() = false;

      previous_signal_strength_[mac] = signal_strength;

    }
  }
  scan_data_->sequence = 0;
  scan_data_->data;

  if(!wifi_data.macs.empty())
  {
    wifi_pub_.publish(wifi_data);
  }
  return true;
}

void WifiPublisher::reset()
{
  wifi_scan_close(channel_);
  delete channel_;
  channel_ = new struct netlink_channel;
  wifi_scan_init(wifi_interface_.c_str(), channel_);
}

unsigned int WifiPublisher::channel_to_freq(unsigned int channel)
{
  if(channel <= 14)
  {
    return 2407 + 5*channel;
  }
  else if(channel <= 167)
  {
    return 5000 + 5*channel;
  }
  else
  {
    // Here we assume that frequencies were given, so that the value does not have to be converted.
    return channel;
  }
}

int WifiPublisher::freq_to_channel(unsigned int frequency)
{

    if (frequency == 2484)
        return 14;

    if (frequency < 2484)
        return (frequency - 2407) / 5;

    return frequency/5 - 1000;
}