//
// Created by 2scholz on 15.08.16.
//

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include "csv_data_loader.h"

CSVDataLoader::CSVDataLoader(std::string path)
{
  std::ifstream file(path);
  int i = -1;
  std::string line;
  while(std::getline(file, line))
  {
    i++;
  }
  file.clear();
  file.seekg(0, std::ios::beg);

  if(i == -1)
    exit(-1);

  observations_matrix_.resize(i,1);
  coordinates_matrix_.resize(i,2);

  std::string value;
  getline(file, value, '\n');

  i = 0;
  std::string x;
  std::string y;
  std::string wifi_signal;
  std::string timestamp;
  std::string channel;
  std::string quat_x;
  std::string quat_y;
  std::string quat_z;
  std::string quat_w;
  while(file.good())
  {
    getline(file, x, ',');
    getline(file, y, ',');
    getline(file, wifi_signal, ',');
    getline(file, timestamp, ',');
    getline(file, channel, ',');
    getline(file, quat_x, ',');
    getline(file, quat_y, ',');
    getline(file, quat_z, ',');
    getline(file, quat_w, '\n');

    if(!file.good())
      break;
    double dx = std::stod(x);
    double dy = std::stod(y);
    double ds = std::stod(wifi_signal);
    coordinates_matrix_(i,0) = dx;
    coordinates_matrix_(i,1) = dy;
    observations_matrix_(i,0) = ds;

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = dx;
    pose.pose.pose.position.y = dy;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation.x = std::stod(quat_x);
    pose.pose.pose.orientation.y = std::stod(quat_y);
    pose.pose.pose.orientation.z = std::stod(quat_z);
    pose.pose.pose.orientation.w = std::stod(quat_w);
    int dt = std::stoi(timestamp);
    int dc = std::stoi(channel);

    data_points_.emplace_back(pose, ds, dt, dc);
    i++;
  }
}