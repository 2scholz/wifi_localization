#include <iostream>
#include <fstream>
#include <ctime>
#include <termios.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Messages
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "wifi_localization/MaxWeight.h"
#include "wifi_localization/WifiState.h"

bool g_record = false;

typedef message_filters::sync_policies::ApproximateTime<wifi_localization::MaxWeight,
  geometry_msgs::PoseWithCovarianceStamped,
  wifi_localization::WifiState> g_sync_policy;

/*
 * Subscriber class
 * The class is used to filter the incoming messages, so that the saved data was approximately generated at the same
 * point in time. The data is written into different csv files, according to the mac address it belongs to.
 */
class Subscriber
{
public:
  Subscriber(ros::NodeHandle &n, double threshold, bool user_input);
  void record_next();

private:
  message_filters::Subscriber<wifi_localization::MaxWeight> *max_weight_sub_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub_;
  message_filters::Subscriber<wifi_localization::WifiState> *wifi_data_sub_;
  message_filters::Synchronizer<g_sync_policy> *sync_;
  double threshold_;
  bool user_input_;
  bool record_;
  std::string date_;
  std::map<std::string, boost::shared_ptr<std::ofstream> > filemap_;

  void callbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg,
                      const wifi_localization::WifiState::ConstPtr &wifi_data_msg);
};

Subscriber::Subscriber(ros::NodeHandle &n, double threshold, bool user_input) :
  threshold_(threshold), user_input_(user_input), record_(false)
{
  ROS_INFO("Threshold at: %f",threshold_);
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
  date_ = buffer;

  boost::filesystem::path dir("./wifi_data/");

  if (!(boost::filesystem::exists(dir)))
  {
    boost::filesystem::create_directory(dir);
  }

  dir = "./wifi_data/" + date_;

  if (!(boost::filesystem::exists(dir)))
  {
    if (boost::filesystem::create_directory(dir))
    {
      ROS_INFO("Created directories to save the data.");
    }
  }

  max_weight_sub_ = new message_filters::Subscriber<wifi_localization::MaxWeight>(n, "max_weight", 100);
  pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n, "amcl_pose", 100);
  wifi_data_sub_ = new message_filters::Subscriber<wifi_localization::WifiState>(n, "wifi_state", 1);

  sync_ = new message_filters::Synchronizer<g_sync_policy>(g_sync_policy(1000), *max_weight_sub_, *pose_sub_,
                                                           *wifi_data_sub_);
  sync_->registerCallback(boost::bind(&Subscriber::callbackMethod, this, _1, _2, _3));
}

void Subscriber::record_next()
{
  record_ = true;
}

//The callback method
void Subscriber::callbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg,
                                const wifi_localization::WifiState::ConstPtr &wifi_data_msg)
{
  ROS_INFO("callback");
  if (max_weight_msg->max_weight > threshold_ && !(user_input_ && !record_))
  {
    ROS_INFO("In loop");
    for (int i = 0; i < wifi_data_msg->macs.size(); i++)
    {
      std::map<std::string, boost::shared_ptr<std::ofstream> >::iterator file = filemap_.find(
        wifi_data_msg->macs.at(i));
      if (file == filemap_.end())
      {
        std::string name = wifi_data_msg->macs.at(i);
        boost::shared_ptr<std::ofstream> new_mac = boost::make_shared<std::ofstream>();
        new_mac->open(std::string("./wifi_data/" + date_ + "/" + name + ".csv").c_str());
        *new_mac << "macs, x, y, strengths, max_weight" << "\n";
        file = filemap_.insert(filemap_.begin(), std::make_pair(name, new_mac));
      }
      *(file->second) << wifi_data_msg->macs.at(i) << "," << pose_msg->pose.pose.position.x << ","
      << pose_msg->pose.pose.position.y << "," << wifi_data_msg->strengths.at(i) << "," << max_weight_msg->max_weight
      << "\n";
    }
    if(record_)
    {
      record_ = false;
      g_record = false;
      ROS_INFO("Recording successful.");
    }
  }
}

// Function that checks for user input.
long getch()
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

  Subscriber *sub = new Subscriber(n, threshold, user_input);

  ros::Rate r(100);
  while (ros::ok())
  {
    if(user_input && !g_record)
    {
      int c = getch();
      if(c == 10)
      {
        ROS_INFO("Record next set of data.");
        g_record = true;
        sub->record_next();
      }
    }
    ros::spinOnce();
    r.sleep();
  }

  delete sub;
  return 0;
}