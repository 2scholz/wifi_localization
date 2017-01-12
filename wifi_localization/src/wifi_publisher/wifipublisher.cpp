#include <wifi_publisher/wifipublisher.h>
#include "wifi_localization/WifiState.h"


WifiPublisher::WifiPublisher(ros::NodeHandle &nh, std::string wifi_infterface, std::vector<float> channel_list):wifi_interface_(wifi_infterface), channel_list_(channel_list)
{
  wifi_socket_ = iw_sockets_open();
  if (wifi_socket_ < 0)
  {
    ROS_ERROR("Failed to open wlan socket on %s", wifi_interface_.c_str());
    ros::shutdown();
  }
  wifi_pub_ = nh.advertise<wifi_localization::WifiState>("wifi_data", 1);

  struct iw_range range;


  /* Get and check range stuff. */
  char* interface_ = new char[wifi_interface_.length() + 1];
  std::strcpy(interface_, wifi_interface_.c_str());
  int has_range = 0;

  has_range = (iw_get_range_info(wifi_socket_, interface_, &range) >= 0);
  if ((!has_range) || (range.we_version_compiled < 14))
  {
    ROS_ERROR("The interface does not support scanning.");
    ros::shutdown();
  }

  /* Set options. */

  /* Clean up set arguments. */
  memset(&scanopt_, 0, sizeof(scanopt_));

  double channel_to_freq;
  iw_freq freq;
  for (int i = 0; i < channel_list_.size(); i++)
  {
    iw_channel_to_freq(channel_list_[i], &channel_to_freq, &range);
    iw_float2freq(channel_to_freq, &freq);
    scanopt_.channel_list[i] = freq;
  }
  scanopt_.num_channels = channel_list_.size();
}


WifiPublisher::~WifiPublisher()
{
  iw_sockets_close(wifi_socket_);
}

bool WifiPublisher::scan()
{
  int scanflags = 0;

  struct iwreq w_request;

  struct iw_range range;

  /* Get and check range stuff. */
  char* interface_ = new char[wifi_interface_.length() + 1];
  std::strcpy(interface_, wifi_interface_.c_str());

  iw_get_range_info(wifi_socket_, interface_, &range);


  w_request.u.data.pointer = (caddr_t)&scanopt_;
  w_request.u.data.length = sizeof(scanopt_);
  w_request.u.data.flags = scanflags;


  if (iw_set_ext(wifi_socket_, wifi_interface_.c_str(), SIOCSIWSCAN, &w_request) < 0)
  {
    return false;
  }

  timeval time;
  time.tv_sec = 0;
  time.tv_usec = 200000;

  uint8_t *p_buff = NULL;
  int buff_size = IW_SCAN_MAX_DATA;

  bool is_end = false;
  while (!is_end)
  {
    fd_set fds;
    int ret;
    FD_ZERO(&fds);
    ret = select(0, &fds, NULL, NULL, &time);
    if (ret == 0)
    {
      uint8_t *p_buff2;
      while (!is_end)
      {
        p_buff2 = (uint8_t *) realloc(p_buff, buff_size);
        p_buff = p_buff2;
        w_request.u.data.pointer = p_buff;
        w_request.u.data.flags = 0;
        w_request.u.data.length = buff_size;
        if (iw_get_ext(wifi_socket_, wifi_interface_.c_str(), SIOCGIWSCAN, &w_request) < 0)
        {
          if (errno == E2BIG && range.we_version_compiled > 16)
          {
            if (w_request.u.data.length > buff_size)
              buff_size = w_request.u.data.length;
            else
              buff_size *= 2;
            continue;
          } else if (errno == EAGAIN)
          {
            time.tv_sec = 0;
            time.tv_usec = 200000;
            break;
          }
        } else
          is_end = true;
      }
    }
  }

  // Put wifi data into ROS message
  wifi_localization::WifiState wifi_data;
  iw_event event;
  stream_descr stream;

  wifi_data.header.stamp = ros::Time::now();

  iw_init_event_stream(&stream, (char *) p_buff, w_request.u.data.length);
  is_end = false;
  int value = 0;
  WifiDataPoint wifi_data_point;
  bool insert_data = false;
  while (!is_end)
  {
    value = iw_extract_event_stream(&stream, &event, range.we_version_compiled);
    if (!(value > 0))
    {
      is_end = true;
    } else
    {
      if (event.cmd == IWEVQUAL)
      {
        // quality part of statistics
        if (event.u.qual.level != 0 || (event.u.qual.updated & (IW_QUAL_DBM | IW_QUAL_RCPI)))
        {

          int8_t noise = event.u.qual.noise;
          int8_t strength = event.u.qual.level;
          wifi_data_point.signal_strength = strength;
          wifi_data_point.noise = noise;
        }
      }
      else if (event.cmd == SIOCGIWAP)
      {
        // get access point MAC addresses
        char mac_buff[1024];
        iw_ether_ntop((const struct ether_addr *) &event.u.ap_addr.sa_data, mac_buff);
        wifi_data_point.mac = std::string(mac_buff);
        wifi_data_point.changed_signal_strength = false;
        for(int i=0;i<wifi_data_old_.macs.size();i++)
        {
          if(wifi_data_old_.macs[i] == wifi_data_point.mac)
          {
            if(wifi_data_old_.strengths[i] != wifi_data_point.signal_strength)
            {
              wifi_data_point.changed_signal_strength = true;
              break;
            }
          }
        }
      }
      else if (event.cmd == SIOCGIWESSID)
      {
        // get ESSID
        std::string ssid;
        ssid = std::string((char*)event.u.essid.pointer);
        ssid.pop_back(); // Removes the last character of the string
        wifi_data_point.ssid = ssid;

      }
      else if (event.cmd == SIOCGIWENCODE)
      {
        // get encoding token & mode
      }
      else if (event.cmd == SIOCGIWFREQ)
      {
        // get channel/frequency (Hz)
        int channel = iw_freq_to_channel(iw_freq2float(&event.u.freq),&range);
        if(std::find(channel_list_.begin(),channel_list_.end(),channel)!=channel_list_.end())
        {
          wifi_data_point.channel = channel;

          // Only add the data, if the channel is in the list.
          wifi_data.ssids.push_back(wifi_data_point.ssid);
          wifi_data.macs.push_back(wifi_data_point.mac);
          wifi_data.channels.push_back(wifi_data_point.channel);
          wifi_data.strengths.push_back(wifi_data_point.signal_strength);
          wifi_data.noise.push_back(wifi_data_point.noise);
          wifi_data.changed_signal_strength.push_back(wifi_data_point.changed_signal_strength);
        }

      }
      else if (event.cmd == SIOCGIWRATE)
      {
        // get default bit rate (bps)
      }
      else if (event.cmd == SIOCGIWMODE)
      {

        // get operation mode

      } else if (event.cmd == IWEVCUSTOM)
      {
        // Driver specific ascii string
      } else if (event.cmd == IWEVGENIE)
      {
        //ROS_INFO("command=IWEVGENIE");
      } else
      {
      }
    }
  }
  wifi_data_old_ = wifi_data;
  wifi_pub_.publish(wifi_data);
  return true;
}