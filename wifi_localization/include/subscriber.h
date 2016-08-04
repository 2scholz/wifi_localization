//
// Created by 2scholz on 01.08.16.
//

#ifndef PROJECT_SUBSCRIBER_H
#define PROJECT_SUBSCRIBER_H
// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "wifi_localization/MaxWeight.h"
#include "wifi_localization/WifiState.h"
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include "mapdata.h"
#include "mapcollection.h"

typedef message_filters::sync_policies::ApproximateTime<wifi_localization::MaxWeight,
geometry_msgs::PoseWithCovarianceStamped> g_sync_policy;

/*
 * Subscriber class
 * The class is used to filter the incoming messages, so that the saved data was approximately generated at the same
 * point in time. The data is written into different csv files, according to the mac address it belongs to.
 * The data is also published to occupancy grids according to the mac addresses.
 */
class Subscriber
{
public:
  Subscriber(ros::NodeHandle &n, double threshold, bool user_input, float map_resolution, std::string path_to_csv = "");
  void recordNext();
  bool& isRecording();

private:
  ros::NodeHandle &n_;

  message_filters::Subscriber<wifi_localization::MaxWeight> *max_weight_sub_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub_;
  ros::Subscriber wifi_data_sub_;
  message_filters::Synchronizer<g_sync_policy> *sync_;

  double threshold_;
  bool user_input_;
  bool record_user_input_;
  bool record_;
  bool record_next_;
  double pos_x_;
  double pos_y_;
  double max_weight_;

  // mappings of the csv files and occupancy grids/publishers to the macs of the access points.
  // std::map<std::string, MapData> mac_map_;
  MapCollection maps;
  ros::ServiceServer recording_service;
  ros::ServiceServer record_next_signal_service;

  void amclCallbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                          const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);

  void wifiCallbackMethod(const wifi_localization::WifiState::ConstPtr& wifi_data_msg);

  bool recording(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  bool record_next_signal(std_srvs::Trigger::Response &req, std_srvs::Trigger::Response &res);
};

#endif //PROJECT_SUBSCRIBER_H
