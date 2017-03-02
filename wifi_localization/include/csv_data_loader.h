//
// Created by 2scholz on 15.08.16.
//

#ifndef PROJECT_CSV_DATA_LOADER_H
#define PROJECT_CSV_DATA_LOADER_H
#include <eigen3/Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

struct DataPoint
{
  /// Vector of poses
  geometry_msgs::PoseWithCovarianceStamped pose_;

  /// Vector of signal strengths
  double signal_strength_;

  /// Timestamps of the time the signal was recorded
  int timestamp_;

  /// Channels of the recorded Wi-Fi signal
  int channel_;

  /// The ssid that belongs to the signal
  std::string ssid_;

  DataPoint(geometry_msgs::PoseWithCovarianceStamped pose, double signal_strength, int timestamp, int channel, std::string ssid) :
    pose_(pose), signal_strength_(signal_strength), timestamp_(timestamp), channel_(channel), ssid_(ssid)
  {}
};

/**
 * CSVDataLoader class
 * Provided a path to a csv-file, that contains coordinates and corresponding wifi-signal strength, it parses these
 * values into Eigen-matrices.
 */
class CSVDataLoader
{
public:
  /**
   * Constructor
   * @param path Path to a csv-file, containing coordinates and wifi-signal strengths
   */
  CSVDataLoader(std::string path);

  /// Coordinates matrix
  Eigen::Matrix<double, Eigen::Dynamic, 2> coordinates_matrix_;

  /// Observed wifi-signal strengths matrix
  Eigen::Matrix<double, Eigen::Dynamic, 1> observations_matrix_;

  /// Vector of data points. Contains the coordinates and observations as well, but in different format
  std::vector<DataPoint> data_points_;
};

#endif //PROJECT_CSV_DATA_LOADER_H
