//
// Created by 2scholz on 12.08.16.
//
#include "wifi_position_estimation/wifi_position_estimation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_localization");
  ros::NodeHandle n;
  WifiPositionEstimation wl(n);

  ros::spin();

  return 0;
}