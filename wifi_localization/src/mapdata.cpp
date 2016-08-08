//
// Created by 2scholz on 03.08.16.
//
#include <vector>
#include "mapdata.h"
#include <fstream>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <eigen3/Eigen/Core>

int MapData::global_min_ = 100;
int MapData::global_max_ = 0;
nav_msgs::OccupancyGrid MapData::empty_map_ = nav_msgs::OccupancyGrid();

MapData::MapData(boost::shared_ptr<std::ofstream> file, ros::Publisher local_minmax_interpol_pub,
                 ros::Publisher local_minmax_norm_pub, ros::Publisher global_minmax_interpol_pub,
                 ros::Publisher global_minmax_norm_pub, int local_min, int local_max) :
  file_(file), local_minmax_interpol_pub_(local_minmax_interpol_pub), local_minmax_norm_pub_(local_minmax_norm_pub),
  global_minmax_interpol_pub_(global_minmax_interpol_pub), global_minmax_norm_pub_(global_minmax_norm_pub),
  local_min_(local_min), local_max_(local_max),
  map_(empty_map_.info.width, std::vector<std::vector<int> >(empty_map_.info.height, std::vector<int>(0))),
  normalized_map_(empty_map_.info.width, std::vector<int>(empty_map_.info.height, -1)),
  interpolated_map_(empty_map_.info.width, std::vector<int>(empty_map_.info.height, -1))
{
  gauss_map_ = empty_map_;
}


void MapData::publish_maps()
{
  normalize_map(local_min_, local_max_);
  local_minmax_norm_pub_.publish(convert_to_grid(normalized_map_));
  interpolate_map();
  // gauss_filter();
  local_minmax_interpol_pub_.publish(convert_to_grid(interpolated_map_));
  // local_minmax_interpol_pub_.publish(gauss_map_);
  normalize_map(global_min_, global_max_);
  global_minmax_norm_pub_.publish(convert_to_grid(normalized_map_));
  interpolate_map();
  // gauss_filter();
  global_minmax_interpol_pub_.publish(convert_to_grid(interpolated_map_));
  // global_minmax_interpol_pub_.publish(gauss_map_);
}

void MapData::insert_data(double x, double y, double wifi_signal)
{

  (*file_) << x << "," << y << "," << wifi_signal << "\n";

  file_->flush();

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

  map_.at(grid_x).at(grid_y).push_back(quality);
  // map_[grid_x][grid_y] = quality;
}

void MapData::load_csv_data(std::string path)
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

  std::string value;
  getline(file, value, '\n');
  i = 0;
  std::string x;
  std::string y;
  std::string wifi_signal;
  while(file.good())
  {
    getline(file, x, ',');
    getline(file, y, ',');
    getline(file, wifi_signal, '\n');
    if(!file.good())
      break;
    double dx = std::stod(x);
    double dy = std::stod(y);
    double ds = std::stod(wifi_signal);
    insert_data(dx, dy, ds);
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

void MapData::normalize_map(int min, int max)
{
  // normalized_map_ = map_;

  int min_max_diff = (max - min);
  if(min_max_diff == 0)
  {
    min_max_diff = 1;
  }
  for(int j=0;j<map_.size(); j++)
  {
    for (int k=0;k<map_[j].size(); k++)
    {
      /*
      if(map_[j][k] != -1)
      {
        normalized_map_[j][k] = ((map_[j][k] - min)*100)/min_max_diff;
      }
       */
      if(!map_.at(j).at(k).empty())
      {
        int val = std::round(std::accumulate(map_.at(j).at(k).begin(), map_.at(j).at(k).end(), 0)/double(map_.at(j).at(k).size()));
        normalized_map_[j][k] = ((val - min)*100)/min_max_diff;
      }
    }
  }
}

// Interpolates the normalized map.
void MapData::interpolate_map()
{
  interpolated_map_ = normalized_map_;

  for(int j=0;j<normalized_map_.size(); j++)
  {
    for (int k=0;k<normalized_map_[j].size(); k++)
    {
      if(normalized_map_[j][k] == -1)
      {
        std::vector<std::pair<double, int> > distance_value;
        int l = 1;

        // Search for cells around the empty cell, for cells that contain values.
        do
        {
          insert_grid_distance_value(distance_value, j, k, j+l, k, get_value(j+l,k));
          insert_grid_distance_value(distance_value, j, k, j-l, k, get_value(j-l,k));
          insert_grid_distance_value(distance_value, j, k, j, k+l, get_value(j,k+l));
          insert_grid_distance_value(distance_value, j, k, j, k-l, get_value(j,k-l));
          for(int m = 1; m < l+1; m++)
          {
            insert_grid_distance_value(distance_value, j, k, j-m, k+l, get_value(j-m,k+l));
            insert_grid_distance_value(distance_value, j, k, j+m, k+l, get_value(j+m,k+l));
            insert_grid_distance_value(distance_value, j, k, j-m, k-l, get_value(j-m,k-l));
            insert_grid_distance_value(distance_value, j, k, j+m, k-l, get_value(j+m,k-l));
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
        interpolated_map_[j][k] = new_value;
      }
    }
  }
}

// Uses gauss filter on normalized map.
void MapData::gauss_filter()
{
  interpolated_map_ = normalized_map_;
  grid_map::GridMap grid_map({"original", "filtered"});
  grid_map.setFrameId("map");
  grid_map.setGeometry(grid_map::Length(empty_map_.info.width, empty_map_.info.height), empty_map_.info.resolution);
  grid_map::GridMapRosConverter::fromOccupancyGrid(convert_to_grid(interpolated_map_), "original", grid_map);

  cv::Mat originalImage;
  grid_map::GridMapCvConverter::toImage<unsigned short, 1>(grid_map, "original", CV_8UC1, -1.0f, 100.0, originalImage);

  cv::Mat newImage;
  cv::GaussianBlur(originalImage, newImage, cv::Size(3,3), 0.0);

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(newImage, "blur", grid_map, 0.0, 100.0);
  grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, "blur", 0.0, 100.0, gauss_map_);
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
