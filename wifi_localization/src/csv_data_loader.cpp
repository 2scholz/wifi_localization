//
// Created by 2scholz on 15.08.16.
//

#include <fstream>
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

  observations.resize(i, 1);
  points.resize(i,2);

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
    points(i,0) = dx;
    points(i,1) = dy;
    observations(i,0) = ds;
    i++;
  }
}