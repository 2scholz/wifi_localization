#include <iostream>
#include <fstream>
#include <ctime>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "wifi_localization/MaxWeight.h"
#include "wifi_localization/WifiState.h"
#include "rosbag/bag.h"

typedef message_filters::sync_policies::ApproximateTime<wifi_localization::MaxWeight,
        geometry_msgs::PoseWithCovarianceStamped,
        wifi_localization::WifiState> syncPolicy;

class Subscriber {
public:
  Subscriber(ros::NodeHandle &n, double threshold_);

private:
  message_filters::Subscriber<wifi_localization::MaxWeight> *max_weight_sub;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub;
  message_filters::Subscriber<wifi_localization::WifiState> *wifi_data_sub;
  message_filters::Synchronizer<syncPolicy>* sync;
  double threshold;
  std::string date;
  std::map<std::string, boost::shared_ptr<std::ofstream> > filemap;

  void callbackMethod(const wifi_localization::MaxWeight::ConstPtr& max_weight_msg,
                      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg,
                      const wifi_localization::WifiState::ConstPtr& wifi_data_msg);
};

Subscriber::Subscriber(ros::NodeHandle &n, double threshold_) : threshold(threshold_)
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
  date = buffer;

  boost::filesystem::path dir("./data/");

  if(!(boost::filesystem::exists(dir)))
  {
    boost::filesystem::create_directory(dir);
  }

  dir = "./data/" + date;

  if(!(boost::filesystem::exists(dir)))
  {
    if (boost::filesystem::create_directory(dir))
    {
      ROS_INFO("Created directories to save the data.");
    }
  }

  max_weight_sub = new message_filters::Subscriber<wifi_localization::MaxWeight>(n, "max_weight", 1000);
  pose_sub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n, "amcl_pose", 1000);
  wifi_data_sub = new message_filters::Subscriber<wifi_localization::WifiState>(n, "wifi_state", 1000);

  sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(1000), *max_weight_sub, *pose_sub, *wifi_data_sub);
  sync->registerCallback(boost::bind(&Subscriber::callbackMethod, this, _1, _2, _3));
}

//The callback method
void Subscriber::callbackMethod(const wifi_localization::MaxWeight::ConstPtr& max_weight_msg,
                                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg,
                                const wifi_localization::WifiState::ConstPtr& wifi_data_msg)
{
  if(max_weight_msg->max_weight > threshold)
  {
    for(int i = 0; i<wifi_data_msg->macs.size(); i++)
    {
      std::map<std::string, boost::shared_ptr<std::ofstream> >::iterator file = filemap.find(wifi_data_msg->macs.at(i));
      if(file == filemap.end())
      {
        std::string name = wifi_data_msg->macs.at(i);
        boost::shared_ptr<std::ofstream> new_mac =boost::make_shared<std::ofstream>();
        new_mac->open(std::string("./data/"+ date + "/" +name+".csv").c_str());
        *new_mac << "macs, x, y, strengths, max_weight" << "\n";
        file = filemap.insert(filemap.begin(), std::make_pair(name, new_mac));
      }
      *(file->second) << wifi_data_msg->macs.at(i) << "," << pose_msg->pose.pose.position.x << ","
      << pose_msg->pose.pose.position.y << "," << wifi_data_msg->strengths.at(i) << "," << max_weight_msg->max_weight
      << "\n";
    }
  }
}

int main(int argc, char **argv)
{
  double threshold = 0.7;

  ros::init(argc, argv, "wifi_data_collector");
  ros::NodeHandle n;

  n.param("threshold", threshold, threshold);

  Subscriber* sub = new Subscriber(n, threshold);

  ros::spin();

  delete sub;
  return 0;
}