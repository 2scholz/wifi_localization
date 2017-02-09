#include "wifi_data_collector/subscriber.h"
#include <fstream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <geometry_msgs/Quaternion.h>

Subscriber::Subscriber(ros::NodeHandle &n, float map_resolution) :
  n_(n),
  maps(n),
  record_(false),
  record_next_(false),
  stands_still_(false),
  record_only_stopped_(false),
  threshold_(1.0),
  pose_()
{
  recorded_since_stop.data = false;
  std::string path_to_csv = "";

  pose_.pose.pose.position.x = 0.0;
  pose_.pose.pose.position.y = 0.0;
  pose_.pose.pose.position.z = 0.0;
  pose_.pose.pose.orientation.x = 0.0;
  pose_.pose.pose.orientation.y = 0.0;
  pose_.pose.pose.orientation.z = 0.0;
  pose_.pose.pose.orientation.w = 0.0;

  n.param("/wifi_data_collector/threshold", threshold_, threshold_);
  n.param("/wifi_data_collector/record_only_stopped", record_only_stopped_, record_only_stopped_);
  n.param("/wifi_data_collector/path_to_csv", path_to_csv, path_to_csv);
  n.param("/wifi_data_collector/record_wifi_signals", record_, record_);

  maps.add_csv_data(path_to_csv);

  ROS_INFO("Threshold at: %f",threshold_);

  max_weight_sub_ = new message_filters::Subscriber<wifi_localization::MaxWeight>(n, "max_weight", 100);
  pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n, "amcl_pose", 100);
  wifi_data_sub_ = n.subscribe("wifi_data", 1, &Subscriber::wifiCallbackMethod, this);
  odom_sub_ = n.subscribe("odom",1, &Subscriber::odomCallbackMethod, this);

  recorded_since_stop_pub_ = n.advertise<std_msgs::Bool>("recorded_since_stop", 1, true);

  sync_ = new message_filters::Synchronizer<g_sync_policy>(g_sync_policy(100), *max_weight_sub_, *pose_sub_);
  sync_->registerCallback(boost::bind(&Subscriber::amclCallbackMethod, this, _1, _2));

  recording_service = n.advertiseService("record_wifi_signals", &Subscriber::recording, this);
  record_next_signal_service = n.advertiseService("record_next_wifi_signal", &Subscriber::record_next_signal, this);
}

//The callback method
void Subscriber::amclCallbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                                    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
  pose_ = *pose_msg;
  max_weight_ = max_weight_msg->max_weight;
}

void Subscriber::wifiCallbackMethod(const wifi_localization::WifiState::ConstPtr& wifi_data_msg)
{
  // Only record the data if max_weight is big enough and if user input mode is activated only if the user pressed the key to record.
  if ((((max_weight_ < threshold_ && record_) || (max_weight_ < threshold_ && record_next_)) && (!record_only_stopped_||stands_still_)) && !wifi_data_msg->macs.empty())
  {
    for (int i = 0; i < wifi_data_msg->macs.size(); i++)
    {
      std::string mac_name = wifi_data_msg->macs.at(i);
      double wifi_dbm = wifi_data_msg->strengths.at(i);

      //maps.add_data(0, mac_name, wifi_dbm, 0, pos_x_, pos_y_, 0);
      maps.add_data(wifi_data_msg->header.stamp.sec, mac_name, wifi_dbm, wifi_data_msg->channels.at(i), pose_);
    }
    if(record_next_)
    {
      record_next_ = false;
      ROS_INFO("Recording successful.");
    }
    ROS_INFO("Recorded new data.");
    if(stands_still_)
    {
      recorded_since_stop.data = true;
      recorded_since_stop_pub_.publish(recorded_since_stop);
    }
  }
}

void Subscriber::odomCallbackMethod(const nav_msgs::Odometry::ConstPtr &msg)
{
  stands_still_ =
          msg->twist.twist.linear.x == 0.0 && msg->twist.twist.linear.y == 0.0 && msg->twist.twist.angular.z == 0.0;
  if(!stands_still_)
  {
    recorded_since_stop.data = false;
    recorded_since_stop_pub_.publish(recorded_since_stop);
  }
}

bool Subscriber::recording(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  record_ = req.data;
  res.success = true;
  return true;
}

bool Subscriber::record_next_signal(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
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