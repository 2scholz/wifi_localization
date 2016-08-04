//
// Created by 2scholz on 01.08.16.
//

#include "subscriber.h"
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_cv/grid_map_cv.hpp>
#include <fstream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
// #include <cv_bridge/cv_bridge.h>

Subscriber::Subscriber(ros::NodeHandle &n, double threshold, bool user_input, float map_resolution, std::string path_to_csv) :
  threshold_(threshold), user_input_(user_input), record_user_input_(false), n_(n), maps(n, path_to_csv), record_(false), record_next_(false)
{
  ROS_INFO("Threshold at: %f",threshold_);

  max_weight_sub_ = new message_filters::Subscriber<wifi_localization::MaxWeight>(n, "max_weight", 100);
  pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n, "amcl_pose", 100);
  wifi_data_sub_ = n.subscribe("wifi_state", 1, &Subscriber::wifiCallbackMethod, this);

  sync_ = new message_filters::Synchronizer<g_sync_policy>(g_sync_policy(100), *max_weight_sub_, *pose_sub_);
  sync_->registerCallback(boost::bind(&Subscriber::amclCallbackMethod, this, _1, _2));

  recording_service = n.advertiseService("start_stop_recording", &Subscriber::recording, this);
  record_next_signal_service = n.advertiseService("record_next_signal", &Subscriber::record_next_signal, this);
}

void Subscriber::recordNext()
{
  record_user_input_ = true;
}

bool& Subscriber::isRecording()
{
  return record_user_input_;
}

//The callback method
void Subscriber::amclCallbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                                    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
  pos_x_ = pose_msg->pose.pose.position.x;
  pos_y_ = pose_msg->pose.pose.position.y;
  max_weight_ = max_weight_msg->max_weight;
}

void Subscriber::wifiCallbackMethod(const wifi_localization::WifiState::ConstPtr& wifi_data_msg)
{
  // Only record the data if max_weight is big enough and if user input mode is activated only if the user pressed the key to record.
  if ((max_weight_ > threshold_ && !(user_input_ && !record_user_input_)) || (max_weight_ > threshold_ && record_) || (max_weight_ > threshold_ && record_next_))
  {
    for (int i = 0; i < wifi_data_msg->macs.size(); i++)
    {
      std::string mac_name = wifi_data_msg->macs.at(i);
      double wifi_dbm = wifi_data_msg->strengths.at(i);

      maps.add_data(mac_name, pos_x_, pos_y_, wifi_dbm);

      /*
      std::map<std::string, MapData>::iterator data = mac_map_.find(mac_name);

      // If there is no entry for this mac address yet, it is going to be created.
      if(data == mac_map_.end())
      {
        boost::shared_ptr<std::ofstream> new_mac = boost::make_shared<std::ofstream>();
        new_mac->open(std::string("./wifi_data/" + date_ + "/" + mac_name + ".csv").c_str());
        *new_mac << "x, y, strengths" << "\n";

        // ros won't allow topics with ':' in them, so they have to be replaced in the mac address.
        std::string mac_pub_name = "/" + mac_name;
        std::replace( mac_pub_name.begin(), mac_pub_name.end(), ':', '_');

        ros::Publisher wifi_map_pub = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_li", 1000, true);
        ros::Publisher wifi_map_pub_2 = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_ln", 1000, true);
        ros::Publisher wifi_map_pub_3 = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_gi", 1000, true);
        ros::Publisher wifi_map_pub_4 = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_gn", 1000, true);

        MapData temp(new_mac, wifi_map_pub, wifi_map_pub_2, wifi_map_pub_3, wifi_map_pub_4);
        data = mac_map_.insert(mac_map_.begin(), std::make_pair(mac_name, temp));
      }

      data->second.insert_data(pos_x_, pos_y_, wifi_dbm);
    */
    }
    if(record_user_input_)
    {
      record_user_input_ = false;
      ROS_INFO("Recording successful.");
    }
    if(record_next_)
    {
      record_next_ = false;
      ROS_INFO("Recording successful.");
    }
  }
}

bool Subscriber::recording(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  record_ = req.data;
  res.success = true;
  return true;
}

bool Subscriber::record_next_signal(std_srvs::Trigger::Response &req, std_srvs::Trigger::Response &res)
{
  if(record_ = true)
  {
    res.success = false;
    res.message = "Service failed, because the node is already recording.";
    return false;
  }
  record_next_ = true;
  res.success = true;
  return true;
}