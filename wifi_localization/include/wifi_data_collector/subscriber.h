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
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include "catkin_ws/src/tams_wifi_localization/wifi_localization/include/wifi_data_collector/mapdata.h"
#include "catkin_ws/src/tams_wifi_localization/wifi_localization/include/wifi_data_collector/mapcollection.h"
#include <sound_play/sound_play.h>


typedef message_filters::sync_policies::ApproximateTime<wifi_localization::MaxWeight,
geometry_msgs::PoseWithCovarianceStamped> g_sync_policy;

/**
 * Subscriber class
 * The class is used to filter the incoming messages, so that the saved data was approximately generated at the same
 * point in time.
 */
class Subscriber
{
public:
  /**
   * Constructor
   * @param n ros NodeHandle
   * @param map_resolution Size of each cell side in meters
   */
  Subscriber(ros::NodeHandle &n, float map_resolution);

private:
  /// ros NodeHandle that is used for subscribers and publishers
  ros::NodeHandle &n_;

  /// Filter for the max_weights to synchronize them with the poses
  message_filters::Subscriber<wifi_localization::MaxWeight> *max_weight_sub_;

  /// Filter for the poses to synchronize them with the max weights
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub_;

  /// Subscriber for the wifi data
  ros::Subscriber wifi_data_sub_;

  /// Subscriber for odometry data. Used to check if the robot is stopped.
  ros::Subscriber odom_sub_;

  /// Publishes if the robot recorded data since the robot stopped.
  ros::Publisher recorded_since_stop_pub_;

  /// Synchronizes the poses and max weights, both sent by amcl
  message_filters::Synchronizer<g_sync_policy> *sync_;

  /// Data is only recorded when the incoming max weight is higher than the threshold
  double threshold_;

  /// Record only when the robot is stopped when set to true
  bool record_only_stopped_;

  /// Subscriber objects only record data when this variable is set to true
  bool record_;

  /// Record the next data point
  bool record_next_;

  /// Is the robot stopped?
  bool stands_still_;

  /// Did the robot record data since it last stopped?
  std_msgs::Bool recorded_since_stop;

  /// Last received x position
  double pos_x_;

  /// Last received y position
  double pos_y_;

  /// Last received max weight. This variable is an indication of the quality and accuracy of amcl
  double max_weight_;

  /// Collection of the maps for all the different mac addresses
  MapCollection maps;

  /// Service that can be called to either stop or start recording data
  ros::ServiceServer recording_service;

  /// Records the next signal when called
  ros::ServiceServer record_next_signal_service;

  /**
   * Used by the synchronized subscriber for the amcl pose and max weight.
   * @param max_weight_msg
   * @param pose_msg
   */
  void amclCallbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                          const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);

  /**
   * Checks if the state of the object allows for recordings and if so adds the incoming wifi data and the latest
   * position estimations to the right maps.
   * @param wifi_data_msg wifi data
   */
  void wifiCallbackMethod(const wifi_localization::WifiState::ConstPtr& wifi_data_msg);

  /**
   * Checks if the robot is stopped.
   * @param msg Odometry data of the turtlebot
   */
  void odomCallbackMethod(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * Used to either start or stop recording via a service call
   * @param req True to start, false to stop recording
   * @param res Indication of success
   * @return Indication of success
   */
  bool recording(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
   * Triggers the recording of the next incoming wifi data
   * @param req Empty
   * @param res Indication of success
   * @return Indication of success
   */
  bool record_next_signal(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

#endif //PROJECT_SUBSCRIBER_H
