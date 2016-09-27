//
// Created by 2scholz on 03.08.16.
//

#ifndef PROJECT_MAPCOLLECTION_H
#define PROJECT_MAPCOLLECTION_H
#include "mapdata.h"
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>

/**
 * Collection of all maps of the different mac addresses.
 */
class MapCollection
{
public:
  /**
   * Constructor
   * @param n ros NodeHandle
   * @param path_to_csv if wished csv data can be loaded when creating the object
   */
  MapCollection(ros::NodeHandle &n, std::string path_to_csv = "");

  /**
   * add data to a map
   * @param mac mac address
   * @param x x position
   * @param y y position
   * @param wifi_signal wifi signal
   */
  void add_data(std::string mac, double x, double y, double wifi_signal);

  /**
   * Add csv data to the maps.
   * @param path Path to folder containing csv files
   */
  void add_csv_data(std::string path);

private:
  /// Holds the mac addresses corresponding to the right map
  std::map<std::string, MapData> mac_map_;

  /// Ros NodeHandle
  ros::NodeHandle n_;

  /// date that is used to create the folder containing the csv files
  std::string date_;

  /**
   * Returns the map matching to the mac address. If such a map does not exist yet it is going to be created.
   * @param mac mac address
   * @return map matching the mac address
   */
  std::map<std::string, MapData>::iterator get_map(std::string mac);

  /// service that publishes the wifi maps
  ros::ServiceServer wifi_map_service;

  /**
   * Publishes all wifi maps
   * @param req empty
   * @param res empty
   * @return Indicates success
   */
  bool publish_map_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

#endif //PROJECT_MAPCOLLECTION_H
