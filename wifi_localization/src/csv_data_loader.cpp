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

  observations_.resize(i,1);
  points_.resize(i,2);

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
    points_(i,0) = dx;
    points_(i,1) = dy;
    observations_(i,0) = ds;
    i++;
  }
}