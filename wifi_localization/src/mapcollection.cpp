//
// Created by 2scholz on 03.08.16.
//
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include <std_srvs/Empty.h>
#include "mapcollection.h"

MapCollection::MapCollection(ros::NodeHandle &n, std::string path_to_csv) : n_(n)
{
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
  date_ = buffer;

  boost::filesystem::path dir("./wifi_data/");

  if (!(boost::filesystem::exists(dir)))
  {
    boost::filesystem::create_directory(dir);
  }

  dir = "./wifi_data/" + date_;

  if (!(boost::filesystem::exists(dir)))
  {
    if (boost::filesystem::create_directory(dir))
    {
      ROS_INFO("Created directories to save the data.");
    }
  }

  wifi_map_service = n.advertiseService("publish_wifi_maps", &MapCollection::publish_map_service, this);

  add_csv_data(path_to_csv);

}

void MapCollection::add_data(std::string mac, double x, double y, double wifi_signal)
{
  std::map<std::string, MapData>::iterator data = get_map(mac);
  data->second.insert_data(x, y, wifi_signal);
}

void MapCollection::add_csv_data(std::string path)
{
  if(path.empty())
    return;
  for(boost::filesystem::directory_iterator itr(path); itr!=boost::filesystem::directory_iterator(); ++itr)
  {
    if(is_regular_file(itr->status()))
    {
      std::string file_path = path+"/"+itr->path().filename().generic_string();
      std::string mac = file_path.substr( file_path.find_last_of("/") + 1 );
      mac = mac.substr(0, mac.find_last_of("."));
      std::map<std::string, MapData>::iterator data = get_map(mac);
      data->second.load_csv_data(file_path);
    }
  }
  ROS_INFO("Finished loading from csv-files.");
}

std::map<std::string, MapData>::iterator MapCollection::get_map(std::string mac)
{
  std::map<std::string, MapData>::iterator data = mac_map_.find(mac);

  // If there is no entry for this mac address yet, it is going to be created.
  if(data == mac_map_.end())
  {
    boost::shared_ptr<std::ofstream> new_mac = boost::make_shared<std::ofstream>();
    new_mac->open(std::string("./wifi_data/" + date_ + "/" + mac + ".csv").c_str());
    *new_mac << "x, y, strengths" << "\n";

    // ros won't allow topics with ':' in them, so they have to be replaced in the mac address.
    std::string mac_pub_name = "/" + mac;
    std::replace( mac_pub_name.begin(), mac_pub_name.end(), ':', '_');

    ros::Publisher wifi_map_pub = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_li", 1000, true);
    ros::Publisher wifi_map_pub_2 = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_ln", 1000, true);
    ros::Publisher wifi_map_pub_3 = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_gi", 1000, true);
    ros::Publisher wifi_map_pub_4 = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"_gn", 1000, true);

    MapData temp(new_mac, wifi_map_pub, wifi_map_pub_2, wifi_map_pub_3, wifi_map_pub_4);
    data = mac_map_.insert(mac_map_.begin(), std::make_pair(mac, temp));
  }
  return data;
}

bool MapCollection::publish_map_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for(std::map<std::string, MapData>::iterator i = mac_map_.begin(); i != mac_map_.end(); i++)
  {
    i->second.publish_maps();
  }
  return true;
}