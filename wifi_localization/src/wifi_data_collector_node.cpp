#include "subscriber.h"
#include <termios.h>

// Messages
#include <nav_msgs/GetMap.h>

// Function that checks for user input.
char getch()
{
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios old = {0};
  if (tcgetattr(filedesc, &old) < 0)
    ROS_ERROR("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &old) < 0)
    ROS_ERROR("tcsetattr ICANON");

  if(rv == -1)
    ROS_ERROR("select");
  else
    read(filedesc, &buff, len );

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
    ROS_ERROR ("tcsetattr ~ICANON");
  return (buff);
}

int main(int argc, char **argv)
{
  bool user_input = false;
  double threshold = 0.0;
  std::string path_to_csv = "";

  ros::init(argc, argv, "wifi_data_collector");
  ros::NodeHandle n;

  n.param("/wifi_data_collector/threshold", threshold, threshold);
  n.param("/wifi_data_collector/user_input", user_input, user_input);
  n.param("/wifi_data_collector/path_to_csv", path_to_csv, path_to_csv);

  std::cout << path_to_csv << std::endl;

  nav_msgs::GetMap srv_map;

  ros::ServiceClient map_service_client = n.serviceClient<nav_msgs::GetMap>("static_map");

  nav_msgs::OccupancyGrid amcl_map_;

  if (map_service_client.call(srv_map))
  {
    ROS_INFO("Map service called successfully");
    amcl_map_ = srv_map.response.map;
  }
  else
  {
    ROS_ERROR("Failed to call map service");
    return 0;
  }
  MapData::empty_map_ = amcl_map_;
  MapData::empty_map_.info.resolution = 0.20;
  MapData::empty_map_.info.width = (unsigned int)((amcl_map_.info.width * amcl_map_.info.resolution)/MapData::empty_map_.info.resolution);
  MapData::empty_map_.info.height = (unsigned int)((amcl_map_.info.height * amcl_map_.info.resolution)/MapData::empty_map_.info.resolution);
  MapData::empty_map_.data.resize(MapData::empty_map_.info.width * MapData::empty_map_.info.height);

  std::fill(MapData::empty_map_.data.begin(), MapData::empty_map_.data.end(), -1);

  Subscriber *sub = new Subscriber(n, threshold, user_input, 0.20, path_to_csv);

  // If user-input mode is deactivated there is no need to check for user input.
  if(!user_input)
  {
    ros::spin();
  }
  else
  {
    ros::Rate r(100);

    while (ros::ok())
    {
      if(!sub->isRecording())
      {
        int c = getch();
        if(c == 10)
        {
          ROS_INFO("Record next set of data.");
          sub->recordNext();
        }
      }
      ros::spinOnce();
      r.sleep();
    }
  }

  delete sub;
  return 0;
}