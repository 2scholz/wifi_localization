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
#include "wifi_data_collector/mapcollection.h"
#include <sys/stat.h>
#include <ios>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

MapCollection::MapCollection(ros::NodeHandle &n, std::string path_to_csv) : n_(n), store_data_separately_(false)
{
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];
  n.param("/wifi_data_collector/store_data_separately", store_data_separately_, store_data_separately_);

  dir_ = "./wifi_data/";

  boost::filesystem::path dir(dir_);

  if (!(boost::filesystem::exists(dir)))
  {
    boost::filesystem::create_directory(dir);
  }

  if(!path_to_csv.empty())
  {
    dir_ = path_to_csv;
    dir = dir_;
    if (!(boost::filesystem::exists(dir)))
    {
      boost::filesystem::create_directory(dir);
    }
  }
  else if(store_data_separately_)
  {
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    std::string date;

    strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
    date = buffer;

    dir_ = dir_ + date + "/";

    dir = dir_;

    if (!(boost::filesystem::exists(dir)))
    {
      if (boost::filesystem::create_directory(dir))
      {
        ROS_INFO("Created directories to store the data separately.");
      }
    }
  }
  else
  {
    dir_="./wifi_data/general/";
    dir = dir_;
    if (!(boost::filesystem::exists(dir)))
    {
      boost::filesystem::create_directory(dir);
    }
  }


  wifi_map_service = n.advertiseService("publish_wifi_maps", &MapCollection::publish_map_service, this);

  add_csv_data(path_to_csv);

}

void MapCollection::add_data(int timestamp, std::string mac, double wifi_signal, int channel,
                             geometry_msgs::PoseWithCovarianceStamped pose)
{
  std::map<std::string, MapData>::iterator data = get_map(mac);
  data->second.insert_data(timestamp, wifi_signal, channel, pose);
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
    std::string filename = dir_ + mac + ".csv";
    bool new_file = true;

    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
      new_file = false;
    }
    new_mac->open(filename.c_str(), std::ios::out | std::ios::app);

    if(new_file)
      *new_mac << "x, y, strengths, timestamp, channel, quat_x, quat_y, quat_z, quat_w" << "\n";

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
