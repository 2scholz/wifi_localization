//
// Created by 2scholz on 03.08.16.
//

#ifndef PROJECT_MAPCOLLECTION_H
#define PROJECT_MAPCOLLECTION_H
#include "mapdata.h"
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>

class MapCollection
{
public:
  MapCollection(ros::NodeHandle &n, std::string path_to_csv = "");
  void add_data(std::string mac, double x, double y, double wifi_signal);
  void add_csv_data(std::string path);

private:
  std::map<std::string, MapData> mac_map_;
  ros::NodeHandle n_;
  std::string date_;
  std::map<std::string, MapData>::iterator get_map(std::string mac);
  ros::ServiceServer wifi_map_service;
  bool publish_map_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

#endif //PROJECT_MAPCOLLECTION_H
