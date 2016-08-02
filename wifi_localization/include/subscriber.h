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

typedef message_filters::sync_policies::ApproximateTime<wifi_localization::MaxWeight,
geometry_msgs::PoseWithCovarianceStamped> g_sync_policy;

struct MapData{
  boost::shared_ptr<std::ofstream> file_;
  std::vector<std::vector<int> > map_;
  ros::Publisher pub_;
  ros::Publisher pub2_;
  int min;
  int max;
  int get_value(int x, int y)
  {
    try {
      return map_.at(x).at(y);
    }
    catch (const std::out_of_range& e) {
      return -1;
    }
  }
};

/*
 * Subscriber class
 * The class is used to filter the incoming messages, so that the saved data was approximately generated at the same
 * point in time. The data is written into different csv files, according to the mac address it belongs to.
 * The data is also published to occupancy grids according to the mac addresses.
 */
class Subscriber
{
public:
  Subscriber(ros::NodeHandle &n, double threshold, bool user_input, float map_resolution);
  void recordNext();
  bool& isRecording();
  bool publish_map_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);


private:
  ros::NodeHandle &n_;

  message_filters::Subscriber<wifi_localization::MaxWeight> *max_weight_sub_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub_;
  ros::Subscriber wifi_data_sub_;
  message_filters::Synchronizer<g_sync_policy> *sync_;

  ros::ServiceServer wifi_map_service;

  double threshold_;
  bool user_input_;
  bool record_;
  std::string date_;
  double pos_x_;
  double pos_y_;
  double max_weight_;
  nav_msgs::OccupancyGrid empty_map_;

  // mappings of the csv files and occupancy grids/publishers to the macs of the access points.
  std::map<std::string, MapData> mac_map_;

  void amclCallbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                          const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);

  void wifiCallbackMethod(const wifi_localization::WifiState::ConstPtr& wifi_data_msg);


  double distance(int cell0, int cell1);

  void insert_grid_distance_value(std::vector<std::pair<double, int> > &distance_value, int x1, int y1, int x2, int y2, int value);
};

#endif //PROJECT_SUBSCRIBER_H
