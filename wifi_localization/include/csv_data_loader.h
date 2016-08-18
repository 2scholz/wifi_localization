//
// Created by 2scholz on 15.08.16.
//

#ifndef PROJECT_CSV_DATA_LOADER_H
#define PROJECT_CSV_DATA_LOADER_H
#include <eigen3/Eigen/Core>

class CSVDataLoader
{
public:
  CSVDataLoader(std::string path);
  Eigen::Matrix<double, Eigen::Dynamic, 2> points;
  Eigen::Matrix<double, Eigen::Dynamic, 1> observations;
};

#endif //PROJECT_CSV_DATA_LOADER_H
