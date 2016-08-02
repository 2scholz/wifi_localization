#include "subscriber.h"
#include <termios.h>

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

  ros::init(argc, argv, "wifi_data_collector");
  ros::NodeHandle n;

  n.param("/wifi_data_collector/threshold", threshold, threshold);
  n.param("/wifi_data_collector/user_input", user_input, user_input);

  Subscriber *sub = new Subscriber(n, threshold, user_input, 0.20);

  ros::ServiceServer wifi_maps = n.advertiseService("publish_wifi_maps", &Subscriber::publish_map_service, sub);

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