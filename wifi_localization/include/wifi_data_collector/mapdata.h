//
// Created by 2scholz on 03.08.16.
//

#ifndef PROJECT_MAPDATA_H
#define PROJECT_MAPDATA_H
#include <ros/publisher.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/**
 * The MapData class holds relevant data and provides relevant methods for a map of wifi_signals.
 */
class MapData
{
public:
  /// static global minimum of all maps
  static int global_min_;

  /// static global maximum of all maps
  static int global_max_;

  /// empty map with the size of the original map
  static nav_msgs::OccupancyGrid empty_map_;

public:
  /**
   * Constructor
   * @param file The csv file the data is going to be written to
   * @param local_minmax_interpol_pub publisher for interpolated map with local min and max
   * @param local_minmax_norm_pub publisher for normalized map with local min and max
   * @param global_minmax_interpol_pub publisher for interpolated map with global min and max
   * @param global_minmax_norm_pub publisher for normalized map with global min and max
   * @param grid_map_pub publisher for the grid map
   */
  MapData(  boost::shared_ptr<std::ofstream> file, ros::Publisher local_minmax_interpol_pub,
            ros::Publisher local_minmax_norm_pub, ros::Publisher global_minmax_interpol_pub,
            ros::Publisher global_minmax_norm_pub);

  /**
   * Returns cell value of given map. When the requested cell is out of bounds -1(unknown) is returned
   * @param map map the value should be read from
   * @param x x position of cell
   * @param y y position of cell
   * @return value in cell, or -1 if out of bounds
   */
  int get_value(const std::vector<std::vector<int> > &map, int x, int y)
  {
    try {
      return map.at(x).at(y);
    }
    catch (const std::out_of_range& e) {
      return -1;
    }
  }

  /**
   * Publishes four different maps of the wifi signal strengths.
   */
  void publish_maps();

  /**
   * insert new data into the map
   * @param x x value of the cell
   * @param y y value of the cell
   * @param wifi_signal signal that is going to be added to the cell
   */
  void insert_data(int timestamp, double wifi_signal, int channel, geometry_msgs::PoseWithCovarianceStamped pose);
/**
   * Load data from csv file from old recording sessions
   * @param path Path to the folder containing the csv files
   */
  void load_csv_data(std::string path);

private:
  /// Pointer to the ofstream of the csv file
  boost::shared_ptr<std::ofstream> file_;

  /// map of the wifi signal strengths. This map is not normalized nor interpolated.
  std::vector<std::vector<std::vector<int> > > map_;

  /// Publisher for interpolated map with local min and max
  ros::Publisher local_minmax_interpol_pub_;

  /// Publisher for normalized map with local min and max
  ros::Publisher local_minmax_norm_pub_;

  /// Publisher for interpolated map with global min and max
  ros::Publisher global_minmax_interpol_pub_;

  /// Publisher for normalized map with global min and max
  ros::Publisher global_minmax_norm_pub_;

  /// local minimum, meaning this is the minimum value that was added to this map
  int local_min_;

  /// local maximum, meaning this is the maximum value that was added to this map
  int local_max_;

  /**
   * Returns a normalized version of the map, with the given min and max
   * @param min minimum value the map is normlized according to
   * @param max maximum value the map is normalized according to
   * @return normalized map
   */
  std::vector<std::vector<int> > normalize_map(int min, int max);

  /**
   * interpolates a given normalized map
   * @param normalized_map normalized map
   * @return interpolated map
   */
  std::vector<std::vector<int> > interpolate_map(const std::vector<std::vector<int> > &normalized_map);

  /**
   * Converts a given map from vector to occupancy grid
   * @param map map in vector form
   * @return map as occupancy grid
   */
  nav_msgs::OccupancyGrid convert_to_grid(std::vector<std::vector<int> > map);

  /**
   * Computes distances between cells. This method is used for the interpolation to compute the inverse distance
   * weighting
   * @param distance_value vector holding pairs of distances and corresponding values of the cell
   * @param x1 x position of cell 1
   * @param y1 y position of cell 1
   * @param x2 x position of cell 2
   * @param y2 y position of cell 2
   * @param value value stored in one of the cells
   */
  void insert_grid_distance_value(std::vector<std::pair<double, int> > &distance_value, int x1, int y1, int x2, int y2, int value);

};

#endif //PROJECT_MAPDATA_H
