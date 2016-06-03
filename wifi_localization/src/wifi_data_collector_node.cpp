#include <iostream>
#include <fstream>
#include <ctime>
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
  Subscriber(ros::NodeHandle &n, double threshold);

private:
  message_filters::Subscriber<wifi_localization::MaxWeight> *max_weight_sub_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub_;
  message_filters::Subscriber<wifi_localization::WifiState> *wifi_data_sub_;
  message_filters::Synchronizer<g_sync_policy> *sync_;
  double threshold_;
  std::string date_;
  std::map<std::string, boost::shared_ptr<std::ofstream> > filemap_;

  void callbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg,
                      const wifi_localization::WifiState::ConstPtr &wifi_data_msg);
};

Subscriber::Subscriber(ros::NodeHandle &n, double threshold) : threshold_(threshold)
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

  max_weight_sub_ = new message_filters::Subscriber<wifi_localization::MaxWeight>(n, "max_weight", 1000);
  pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n, "amcl_pose", 1000);
  wifi_data_sub_ = new message_filters::Subscriber<wifi_localization::WifiState>(n, "wifi_state", 1000);

  sync_ = new message_filters::Synchronizer<g_sync_policy>(g_sync_policy(1000), *max_weight_sub_, *pose_sub_,
                                                           *wifi_data_sub_);
  sync_->registerCallback(boost::bind(&Subscriber::callbackMethod, this, _1, _2, _3));
}

//The callback method
void Subscriber::callbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg,
                                const wifi_localization::WifiState::ConstPtr &wifi_data_msg)
{
  if (max_weight_msg->max_weight > threshold_)
  {
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
  }
}

int main(int argc, char **argv)
{
  double threshold = 0.0;

  ros::init(argc, argv, "wifi_data_collector");
  ros::NodeHandle n;

  n.param("threshold", threshold, threshold);

  Subscriber *sub = new Subscriber(n, threshold);

  ros::spin();

  delete sub;
  return 0;
}