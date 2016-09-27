//
// Created by 2scholz on 19.09.16.
//

#ifndef PROJECT_WIFI_POSITION_ESTIMATION_H
#define PROJECT_WIFI_POSITION_ESTIMATION_H
#include <ros/init.h>
#include <ros/node_handle.h>
#include "gaussian_process/gaussian_process.h"
#include "csv_data_loader.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/filesystem.hpp>
#include <std_srvs/Empty.h>
#include <wifi_localization/WifiState.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include <wifi_localization/MaxWeight.h>
#include <wifi_localization/WifiPositionEstimation.h>

using namespace boost::filesystem;

class WifiPositionEstimation
{
public:
  WifiPositionEstimation(ros::NodeHandle &n);

  Eigen::Vector2d random_position();

private:
  int n_particles;
  Eigen::Vector2d A;
  Eigen::Vector2d AB;
  Eigen::Vector2d AC;
  double x_pos;
  double y_pos;
  double quality_threshold_;

  bool computing;
  bool publish_pose_lock_;

  std::map<std::string, Process> gps;
  std::vector<std::pair<std::string, double>> macs_and_strengths;

  ros::Publisher initialpose_pub;
  ros::Publisher wifi_pos_estimation_pub;
  ros::Subscriber wifi_sub;
  ros::Subscriber max_weight_sub_;
  ros::Subscriber amcl_sub;
  ros::ServiceServer compute_starting_point_service;
  ros::ServiceServer publish_accuracy_data_service;

  bool publish_pose_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
  bool publish_accuracy_data(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  geometry_msgs::PoseWithCovarianceStamped compute_pose();
  void wifi_callback(const wifi_localization::WifiState::ConstPtr& msg);
  void max_weight_callback(const wifi_localization::MaxWeight::ConstPtr& msg);
  void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
};
#endif //PROJECT_WIFI_POSITION_ESTIMATION_H
