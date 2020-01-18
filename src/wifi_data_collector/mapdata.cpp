//
// Created by 2scholz on 03.08.16.
//
#include <vector>
#include "wifi_data_collector/mapdata.h"
#include <fstream>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <csv_data_loader.h>


int MapData::global_min_ = 100;
int MapData::global_max_ = 0;
nav_msgs::OccupancyGrid MapData::empty_map_ = nav_msgs::OccupancyGrid();

MapData::MapData(boost::shared_ptr<std::ofstream> file, ros::Publisher local_minmax_interpol_pub,
                 ros::Publisher local_minmax_norm_pub, ros::Publisher global_minmax_interpol_pub,
                 ros::Publisher global_minmax_norm_pub) :
  file_(file), local_minmax_interpol_pub_(local_minmax_interpol_pub), local_minmax_norm_pub_(local_minmax_norm_pub),
  global_minmax_interpol_pub_(global_minmax_interpol_pub), global_minmax_norm_pub_(global_minmax_norm_pub),
  local_min_(100), local_max_(0),
  map_(empty_map_.info.width, std::vector<std::vector<int> >(empty_map_.info.height, std::vector<int>(0)))
{

}


void MapData::publish_maps()
{
  std::vector<std::vector<int> > normalized_map = normalize_map(local_min_, local_max_);
  local_minmax_norm_pub_.publish(convert_to_grid(normalized_map));
  local_minmax_interpol_pub_.publish(convert_to_grid(interpolate_map(normalized_map)));
  normalized_map = normalize_map(global_min_, global_max_);
  global_minmax_norm_pub_.publish(convert_to_grid(normalized_map));
  global_minmax_interpol_pub_.publish(convert_to_grid(interpolate_map(normalized_map)));
}

void MapData::insert_data(int timestamp, double wifi_signal, int channel, geometry_msgs::PoseWithCovarianceStamped pose, std::string ssid, bool add_to_file)
{
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  geometry_msgs::Quaternion orientation = pose.pose.pose.orientation;

  if(add_to_file)
  {
    // Write the data to the csv file
    (*file_) << x << "," << y << "," << wifi_signal << "," << timestamp << "," << channel << "," << orientation.x << "," << orientation.y << "," << orientation.z << "," << orientation.w  << "," << ssid << "\n";

    file_->flush();
  }


  // Find the right cell, to insert the wifi data into.
  int grid_x = (int)((x - empty_map_.info.origin.position.x) / empty_map_.info.resolution);
  int grid_y = (int)((y - empty_map_.info.origin.position.y) / empty_map_.info.resolution);

  // convert the dbm values of the wifi strength to percentage values.
  int8_t quality;

  if(wifi_signal <= -100)
    quality = 0;
  else if(wifi_signal >= -50)
    quality = 100;
  else
    quality = 2 * (int(wifi_signal) + 100);

  if(quality < local_min_)
  {
    local_min_ = quality;

    if(quality < MapData::global_min_)
      MapData::global_min_ = quality;
  }

  else if(quality > local_max_)
  {
    local_max_ = quality;

    if(quality > MapData::global_max_)
      MapData::global_max_ = quality;
  }

  if(grid_x < 0 || grid_y < 0)
    ROS_WARN("grid_x or grid_y out of range");
  else
    map_.at(grid_x).at(grid_y).push_back(quality);
}

void MapData::load_csv_data(std::string path)
{
  CSVDataLoader csv_data(path);
  for(auto data_point:csv_data.data_points_)
  {
    insert_data(data_point.timestamp_, data_point.signal_strength_, data_point.channel_, data_point.pose_, data_point.ssid_, false);
  }
}

void MapData::insert_grid_distance_value(std::vector<std::pair<double, int> > &distance_value, int x1, int y1, int x2, int y2, int value)
{
  if(value != -1)
  {
    double distance = pow(sqrt(pow(x2 - x1,2) + pow(y2 - y1, 2)),2);
    distance_value.push_back(std::make_pair(distance, value));
  }
}

nav_msgs::OccupancyGrid MapData::convert_to_grid(std::vector<std::vector<int> > map)
{
  nav_msgs::OccupancyGrid grid = empty_map_;

  for(int j=0;j<map.size(); j++)
  {
    for (int k=0;k<map[j].size(); k++)
    {
      grid.data.at(j + (k*empty_map_.info.width)) = map[j][k];
    }
  }

  return grid;
}

std::vector<std::vector<int> > MapData::normalize_map(int min, int max)
{
  std::vector<std::vector<int> > normalized_map(empty_map_.info.width, std::vector<int>(empty_map_.info.height, -1));

  int min_max_diff = (max - min);
  if(min_max_diff == 0)
  {
    min_max_diff = 1;
  }
  for(int j=0;j<map_.size(); j++)
  {
    for (int k=0;k<map_[j].size(); k++)
    {
      if(!map_.at(j).at(k).empty())
      {
        int val = std::round(std::accumulate(map_.at(j).at(k).begin(), map_.at(j).at(k).end(), 0)/double(map_.at(j).at(k).size()));
        normalized_map[j][k] = ((val - min)*100)/min_max_diff;
      }
    }
  }
  return normalized_map;
}

std::vector<std::vector<int> > MapData::interpolate_map(const std::vector<std::vector<int> > &normalized_map)
{
  std::vector<std::vector<int> > interpolated_map = normalized_map;

  for(int j=0;j<normalized_map.size(); j++)
  {
    for (int k=0;k<normalized_map[j].size(); k++)
    {
      if(normalized_map[j][k] == -1)
      {
        std::vector<std::pair<double, int> > distance_value;
        int l = 1;

        // Search for cells around the empty cell, for cells that contain values.
        do
        {
          insert_grid_distance_value(distance_value, j, k, j+l, k, get_value(normalized_map, j+l,k));
          insert_grid_distance_value(distance_value, j, k, j-l, k, get_value(normalized_map, j-l,k));
          insert_grid_distance_value(distance_value, j, k, j, k+l, get_value(normalized_map, j,k+l));
          insert_grid_distance_value(distance_value, j, k, j, k-l, get_value(normalized_map, j,k-l));
          for(int m = 1; m < l+1; m++)
          {
            insert_grid_distance_value(distance_value, j, k, j-m, k+l, get_value(normalized_map, j-m,k+l));
            insert_grid_distance_value(distance_value, j, k, j+m, k+l, get_value(normalized_map, j+m,k+l));
            insert_grid_distance_value(distance_value, j, k, j-m, k-l, get_value(normalized_map, j-m,k-l));
            insert_grid_distance_value(distance_value, j, k, j+m, k-l, get_value(normalized_map, j+m,k-l));
          }
          l++;
        }while(l < 3);

        // compute the inverse distance weighting
        double new_value1 = 0;
        double new_value2 = 0;

        for(int m = 0; m < distance_value.size(); m++)
        {
          new_value1 += distance_value.at(m).second/distance_value.at(m).first;
          new_value2 += 1/distance_value.at(m).first;
        }

        int new_value = int(new_value1/new_value2);

        // If there were no occupied cells found nearby, the cell is going to be set to -1 (unknown)
        if(distance_value.empty())
        {
          new_value = -1;
        }

        interpolated_map[j][k] = new_value;
      }
    }
  }
  return interpolated_map;
}