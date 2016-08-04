//
// Created by 2scholz on 03.08.16.
//

#ifndef PROJECT_MAPDATA_H
#define PROJECT_MAPDATA_H
#include <ros/publisher.h>
#include <nav_msgs/OccupancyGrid.h>

class MapData
{
public:
  static int global_min_;
  static int global_max_;
  static nav_msgs::OccupancyGrid empty_map_;

public:
  MapData(  boost::shared_ptr<std::ofstream> file, ros::Publisher local_minmax_interpol_pub,
            ros::Publisher local_minmax_norm_pub, ros::Publisher global_minmax_interpol_pub,
            ros::Publisher global_minmax_norm_pub, int local_min = 100, int local_max = 0);

  int get_value(int x, int y)
  {
    try {
      return normalized_map_.at(x).at(y);
    }
    catch (const std::out_of_range& e) {
      return -1;
    }
  }

  void publish_maps();
  void insert_data(double x, double y, double wifi_signal);
  void load_csv_data(std::string path);

private:
  boost::shared_ptr<std::ofstream> file_;

  std::vector<std::vector<int> > map_;
  std::vector<std::vector<int> > normalized_map_;
  std::vector<std::vector<int> > interpolated_map_;

  ros::Publisher local_minmax_interpol_pub_;
  ros::Publisher local_minmax_norm_pub_;
  ros::Publisher global_minmax_interpol_pub_;
  ros::Publisher global_minmax_norm_pub_;
  int local_min_;
  int local_max_;
  void normalize_map(int min, int max);
  void interpolate_map();
  nav_msgs::OccupancyGrid convert_to_grid(std::vector<std::vector<int> > map);

  void insert_grid_distance_value(std::vector<std::pair<double, int> > &distance_value, int x1, int y1, int x2, int y2, int value);

};

#endif //PROJECT_MAPDATA_H
