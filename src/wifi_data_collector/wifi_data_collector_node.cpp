#include "wifi_data_collector/subscriber.h"
#include <termios.h>

// Messages
#include <nav_msgs/GetMap.h>

/**
 * Creates the NodeHandle and the Subscriber object.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_data_collector");
  ros::NodeHandle n;

  std::string path_to_csv = "";
  n.param("/wifi_data_collector/path_to_csv", path_to_csv, path_to_csv);

  nav_msgs::GetMap srv_map;
  ros::ServiceClient map_service_client = n.serviceClient<nav_msgs::GetMap>("static_map");

  nav_msgs::OccupancyGrid amcl_map_;

  if (map_service_client.call(srv_map))
  {
    ROS_INFO("Map service called successfully");
    amcl_map_ = srv_map.response.map;
  }
  else
  {
    ROS_WARN("Failed to call map service. Using default map instead.");
    amcl_map_.info.map_load_time = ros::Time::now();
    amcl_map_.info.resolution = 0.20;
    amcl_map_.info.height = 1000;
    amcl_map_.info.width = 1000;

    amcl_map_.info.origin.position.x = 0.0;
    amcl_map_.info.origin.position.y = 0.0;
    amcl_map_.info.origin.position.z = 0.0;

    amcl_map_.info.origin.orientation.x = 0.0;
    amcl_map_.info.origin.orientation.y = 0.0;
    amcl_map_.info.origin.orientation.z = 0.0;
    amcl_map_.info.origin.orientation.w = 1.0;
  }
  MapData::empty_map_ = amcl_map_;
  MapData::empty_map_.info.resolution = 0.20;
  MapData::empty_map_.info.width = (unsigned int)((amcl_map_.info.width * amcl_map_.info.resolution)/MapData::empty_map_.info.resolution);
  MapData::empty_map_.info.height = (unsigned int)((amcl_map_.info.height * amcl_map_.info.resolution)/MapData::empty_map_.info.resolution);
  MapData::empty_map_.data.resize(MapData::empty_map_.info.width * MapData::empty_map_.info.height);

  std::fill(MapData::empty_map_.data.begin(), MapData::empty_map_.data.end(), -1);

  Subscriber *sub = new Subscriber(n, 0.20, path_to_csv);

  ros::spin();

  delete sub;
  return 0;
}