//
// Created by 2scholz on 15.08.16.
//

#ifndef PROJECT_CSV_DATA_LOADER_H
#define PROJECT_CSV_DATA_LOADER_H
#include <eigen3/Eigen/Core>

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

  /// Coordinates
  Eigen::Matrix<double, Eigen::Dynamic, 2> points;

  /// Observed wifi-signal strengths
  Eigen::Matrix<double, Eigen::Dynamic, 1> observations;
};

#endif //PROJECT_CSV_DATA_LOADER_H
