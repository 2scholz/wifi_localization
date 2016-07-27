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
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

typedef message_filters::sync_policies::ApproximateTime<wifi_localization::MaxWeight,
  geometry_msgs::PoseWithCovarianceStamped> g_sync_policy;

struct Map_data{
  boost::shared_ptr<std::ofstream> file_;
  nav_msgs::OccupancyGrid map_;
  ros::Publisher pub_;
};

/*
 * Subscriber class
 * The class is used to filter the incoming messages, so that the saved data was approximately generated at the same
 * point in time. The data is written into different csv files, according to the mac address it belongs to.
 * The data is also published to occupancy grids according to the mac addresses.
 */
class Subscriber
{
public:
  Subscriber(ros::NodeHandle &n, double threshold, bool user_input);
  void recordNext();
  bool& isRecording();

private:
  ros::NodeHandle &n_;

  message_filters::Subscriber<wifi_localization::MaxWeight> *max_weight_sub_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub_;
  ros::Subscriber wifi_data_sub_;
  message_filters::Synchronizer<g_sync_policy> *sync_;

  double threshold_;
  bool user_input_;
  bool record_;
  std::string date_;
  double pos_x_;
  double pos_y_;
  double max_weight_;
  nav_msgs::OccupancyGrid amcl_map_;

  // TODO: delete the variables that aren't used.
  int map_height_;
  int map_width_;
  double map_resolution_;
  double map_origin_x_;
  double map_origin_y_;


  // mappings of the csv files and occupancy grids/publishers to the macs of the access points.
  std::map<std::string, boost::shared_ptr<std::ofstream> > filemap_;
  std::map<std::string, std::pair<nav_msgs::OccupancyGrid, ros::Publisher> > wifi_map_pubs_;
  std::map<std::string, Map_data> mac_map_;

  void amclCallbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);

  void wifiCallbackMethod(const wifi_localization::WifiState::ConstPtr& wifi_data_msg);
};

Subscriber::Subscriber(ros::NodeHandle &n, double threshold, bool user_input) :
  threshold_(threshold), user_input_(user_input), record_(false), n_(n)
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
  wifi_data_sub_ = n.subscribe("wifi_state", 1, &Subscriber::wifiCallbackMethod, this);

  sync_ = new message_filters::Synchronizer<g_sync_policy>(g_sync_policy(100), *max_weight_sub_, *pose_sub_);
  sync_->registerCallback(boost::bind(&Subscriber::amclCallbackMethod, this, _1, _2));

  nav_msgs::GetMap srv_map;

  ros::ServiceClient map_service_client = n.serviceClient<nav_msgs::GetMap>("static_map");

  if (map_service_client.call(srv_map))
  {
    ROS_INFO("Map service called successfully");
    amcl_map_ = srv_map.response.map;
  }
  else
  {
    ROS_ERROR("Failed to call map service");
    return;
  }

  //amcl_map_.data.erase(amcl_map_.data.begin(), amcl_map_.data.end());
  std::fill(amcl_map_.data.begin(), amcl_map_.data.end(), 0);
  map_height_ = amcl_map_.info.height;
  map_width_ = amcl_map_.info.width;
  map_resolution_ = amcl_map_.info.resolution;
  map_origin_x_ = amcl_map_.info.origin.position.x;
  map_origin_y_ = amcl_map_.info.origin.position.y;
}

void Subscriber::recordNext()
{
  record_ = true;
}

bool& Subscriber::isRecording()
{
  return record_;
}

//The callback method
void Subscriber::amclCallbackMethod(const wifi_localization::MaxWeight::ConstPtr &max_weight_msg,
                                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
  pos_x_ = pose_msg->pose.pose.position.x;
  pos_y_ = pose_msg->pose.pose.position.y;
  max_weight_ = max_weight_msg->max_weight;
}

void Subscriber::wifiCallbackMethod(const wifi_localization::WifiState::ConstPtr& wifi_data_msg)
{
  // Only record the data if max_weight is big enough and if user input mode is activated only if the user pressed the key to record.
  if (max_weight_ > threshold_ && !(user_input_ && !record_))
  {
    for (int i = 0; i < wifi_data_msg->macs.size(); i++)
    {
      std::string mac_name = wifi_data_msg->macs.at(i);
      double wifi_dbm = wifi_data_msg->strengths.at(i);

      // std::map<std::string, boost::shared_ptr<std::ofstream> >::iterator file = filemap_.find(mac_name);

      // std::map<std::string, std::pair<nav_msgs::OccupancyGrid, ros::Publisher> >::iterator map = wifi_map_pubs_.find(mac_name);

      std::map<std::string, Map_data>::iterator data = mac_map_.find(mac_name);

      // If the file for this mac address doesn't exist yet, it is going to be created.
      /*
      if (file == filemap_.end())
      {
        boost::shared_ptr<std::ofstream> new_mac = boost::make_shared<std::ofstream>();
        new_mac->open(std::string("./wifi_data/" + date_ + "/" + mac_name + ".csv").c_str());
        *new_mac << "x, y, strengths" << "\n";
        file = filemap_.insert(filemap_.begin(), std::make_pair(mac_name, new_mac));

        ros::Publisher wifi_map_pub = n_.advertise<nav_msgs::OccupancyGrid>(mac_name, 1000);
        map = wifi_map_pubs_.insert(wifi_map_pubs_.begin(),
                                    std::make_pair(mac_name, std::make_pair(amcl_map_, wifi_map_pub)));
      }
      */
      if(data == mac_map_.end())
      {
        boost::shared_ptr<std::ofstream> new_mac = boost::make_shared<std::ofstream>();
        new_mac->open(std::string("./wifi_data/" + date_ + "/" + mac_name + ".csv").c_str());
        *new_mac << "x, y, strengths" << "\n";
        // file = filemap_.insert(filemap_.begin(), std::make_pair(mac_name, new_mac));

        ros::Publisher wifi_map_pub = n_.advertise<nav_msgs::OccupancyGrid>(mac_name, 1000);
        // map = wifi_map_pubs_.insert(wifi_map_pubs_.begin(), std::make_pair(mac_name, std::make_pair(amcl_map_, wifi_map_pub)));

        Map_data temp;
        temp.file_ = new_mac;
        temp.map_ = amcl_map_;
        temp.pub_ = wifi_map_pub;
        data = mac_map_.insert(mac_map_.begin(), std::make_pair(mac_name, temp));
      }

      // *(file->second) << pos_x_ << "," << pos_y_ << "," << wifi_dbm << "\n";

      *(data->second).file_ << pos_x_ << "," << pos_y_ << "," << wifi_dbm << "\n";

      //file->second->flush();

      data->second.file_->flush();

      // Find the right cell, to insert the wifi data into.
      // std::vector<int8_t>::iterator cell = map->second.first.data.begin();
      std::vector<int8_t>::iterator cell = data->second.map_.data.begin();
      std::advance(cell, int((pos_x_/map_resolution_) + (pos_y_*map_width_)));

      // convert the dbm values of the wifi strength to percentage values.
      int quality;

      if(wifi_dbm <= -100)
        quality = 0;
      else if(wifi_dbm >= -50)
        quality = 100;
      else
        quality = 2 * (int(wifi_dbm) + 100);

      // insert the wifi-signal-quality into the occupancy grid
      // map->second.first.data.insert(cell, quality);
      data->second.map_.data.insert(cell, quality);

      // publish the occupancy grid
      // map->second.second.publish(map->second.first);
      data->second.pub_.publish(data->second.map_);

    }
    if(record_)
    {
      record_ = false;
      ROS_INFO("Recording successful.");
    }
  }
}

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

  Subscriber *sub = new Subscriber(n, threshold, user_input);

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