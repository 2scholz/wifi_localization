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

/**
 * WifiPositionEstimation class
 * Given a set of wifi-signal strength with the corresponding mac-addresses, it approximates the position of the
 * robot. This is done by randomly spreading particles over the entire map, then computing the probability on each
 * position and at the end choosing the particle with the highest probability.
 */
class WifiPositionEstimation
{
public:
  /**
   * Constructor
   * @param n ros nodehandle
   */
  WifiPositionEstimation(ros::NodeHandle &n);

  /**
   * Computes a random position on the current map.
   * @return random position as Vector
   */
  Eigen::Vector2d random_position();

private:
  /// Number of particles used for the position estimation
  int n_particles;

  /// Vectors used in computing a random position on the map. These vectors are corners of the map.
  Eigen::Vector2d A;
  Eigen::Vector2d AB;
  Eigen::Vector2d AC;

  /// x coordinate provided by amcl
  double x_pos;

  /// y coordinate provided by amcl
  double y_pos;

  /// When max_weight from amcl, exceeds this threshold, the wifi position estimation is started.
  double quality_threshold_;

  /// Signals that a new position is already estimated, to stop it from starting multiple estimations.
  bool computing;

  /// map of macs and corresponding Gaussian processes.
  std::map<std::string, Process> gps;

  /// Vector of incoming signal strengths and the corresponding mac-addresses.
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
  void wifi_callback(const wifi_localization::WifiState::ConstPtr& msg);
  void max_weight_callback(const wifi_localization::MaxWeight::ConstPtr& msg);
  void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /**
   * Computes the most likely pose.
   * @return Most likely pose.
   */
  geometry_msgs::PoseWithCovarianceStamped compute_pose();
};
#endif //PROJECT_WIFI_POSITION_ESTIMATION_H
