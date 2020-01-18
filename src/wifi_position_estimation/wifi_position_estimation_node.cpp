//
// Created by 2scholz on 12.08.16.
//
#include "wifi_position_estimation/wifi_position_estimation.h"

/**
 * Node for the wifi_position_estimation.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_localization");
  ros::NodeHandle n;
  WifiPositionEstimation wl(n);
  bool publish_data_periodically = false;
  double periodic_publishing_rate = 0.10;
  std_srvs::Empty::Request req; 
  std_srvs::Empty::Response res;

  n.param("/wifi_position_estimation/publish_data_periodically", publish_data_periodically, publish_data_periodically);
  n.param("/wifi_position_estimation/periodic_publishing_rate", periodic_publishing_rate, periodic_publishing_rate);

  if(publish_data_periodically)
  {
    ros::Rate r(periodic_publishing_rate);
    while (ros::ok())
    {
      wl.publish_accuracy_data(req, res);
      ros::spinOnce();
      r.sleep();
    }
  }
  else
    ros::spin();
  

  return 0;
}
