//
// Created by 2scholz on 01.08.16.
//

#include "subscriber.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <fstream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <cv_bridge/cv_bridge.h>

// Messages
#include <nav_msgs/GetMap.h>

Subscriber::Subscriber(ros::NodeHandle &n, double threshold, bool user_input, float map_resolution) :
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

  nav_msgs::OccupancyGrid amcl_map_;

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
  empty_map_ = amcl_map_;
  empty_map_.info.resolution = map_resolution;
  empty_map_.info.width = (unsigned int)((amcl_map_.info.width * amcl_map_.info.resolution)/empty_map_.info.resolution);
  empty_map_.info.height = (unsigned int)((amcl_map_.info.height * amcl_map_.info.resolution)/empty_map_.info.resolution);
  empty_map_.data.resize(empty_map_.info.width * empty_map_.info.height);

  std::fill(empty_map_.data.begin(), empty_map_.data.end(), -1);
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

      std::map<std::string, MapData>::iterator data = mac_map_.find(mac_name);

      // If there is no entry for this mac address yet, it is going to be created.
      if(data == mac_map_.end())
      {
        boost::shared_ptr<std::ofstream> new_mac = boost::make_shared<std::ofstream>();
        new_mac->open(std::string("./wifi_data/" + date_ + "/" + mac_name + ".csv").c_str());
        *new_mac << "x, y, strengths" << "\n";

        // ros won't allow topics with ':' in them, so they have to be replaced in the mac address.
        std::string mac_pub_name = "/" + mac_name;
        std::replace( mac_pub_name.begin(), mac_pub_name.end(), ':', '_');

        ros::Publisher wifi_map_pub = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name, 1000, true);
        ros::Publisher wifi_map_pub_alt = n_.advertise<nav_msgs::OccupancyGrid>(mac_pub_name+"scld", 1000, true);

        std::vector<std::vector<int> > map(empty_map_.info.width, std::vector<int>(empty_map_.info.height, -1));


        MapData temp = {
          new_mac,
          map,
          wifi_map_pub,
          wifi_map_pub_alt,
          100,
          0
        };
        data = mac_map_.insert(mac_map_.begin(), std::make_pair(mac_name, temp));
      }

      *(data->second).file_ << pos_x_ << "," << pos_y_ << "," << wifi_dbm << "\n";

      data->second.file_->flush();

      // Find the right cell, to insert the wifi data into.
      int grid_x = (int)((pos_x_ - empty_map_.info.origin.position.x) / empty_map_.info.resolution);
      int grid_y = (int)((pos_y_ - empty_map_.info.origin.position.y) / empty_map_.info.resolution);

      // convert the dbm values of the wifi strength to percentage values.
      int8_t quality;

      if(wifi_dbm <= -100)
        quality = 0;
      else if(wifi_dbm >= -50)
        quality = 100;
      else
        quality = 2 * (int(wifi_dbm) + 100);

      if(quality < data->second.min)
        data->second.min = quality;
      else if(quality > data->second.max)
        data->second.max = quality;

      data->second.map_[grid_x][grid_y] = quality;
    }
    if(record_)
    {
      record_ = false;
      ROS_INFO("Recording successful.");
    }
  }
}

bool Subscriber::publish_map_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for(std::map<std::string, MapData>::iterator i = mac_map_.begin(); i != mac_map_.end(); i++)
  {
    // Create a new grid and scale the numbers to 0 - 100 according to their min and max values.
    MapData scaled_map = i->second;


    int min_max_diff = (i->second.max - i->second.min);
    if((i->second.max - i->second.min) == 0)
    {
      min_max_diff = 1;
    }
    for(int j=0;j<scaled_map.map_.size(); j++)
    {
      for (int k=0;k<scaled_map.map_[j].size(); k++)
      {
        if(scaled_map.map_[j][k] != -1)
        {
          scaled_map.map_[j][k] = ((scaled_map.map_[j][k] - i->second.min)*100)/min_max_diff;
        }
      }
    }

    // Create an interpolated map.
    MapData interpol_map = scaled_map;


    for(int j=0;j<scaled_map.map_.size(); j++)
    {
      for (int k=0;k<scaled_map.map_[j].size(); k++)
      {
        if(scaled_map.map_[j][k] == -1)
        {
          std::vector<std::pair<double, int> > distance_value;
          int l = 1;

          // Search for cells around the empty cell, for cells that contain values.
          do
          {
            insert_grid_distance_value(distance_value, j, k, j+l, k, scaled_map.get_value(j+l,k));
            insert_grid_distance_value(distance_value, j, k, j-l, k, scaled_map.get_value(j-l,k));
            insert_grid_distance_value(distance_value, j, k, j, k+l, scaled_map.get_value(j,k+l));
            insert_grid_distance_value(distance_value, j, k, j, k-l, scaled_map.get_value(j,k-l));
            for(int m = 1; m < l+1; m++)
            {
              insert_grid_distance_value(distance_value, j, k, j-m, k+l, scaled_map.get_value(j-m,k+l));
              insert_grid_distance_value(distance_value, j, k, j+m, k+l, scaled_map.get_value(j+m,k+l));
              insert_grid_distance_value(distance_value, j, k, j-m, k-l, scaled_map.get_value(j-m,k-l));
              insert_grid_distance_value(distance_value, j, k, j+m, k-l, scaled_map.get_value(j+m,k-l));
            }
            l++;
          }while(l < 3);

          // compute the inverse distance weighting
          double new_value1 = 0;
          double new_value2 = 0;
          for(int m = 0; m < distance_value.size(); m++)
          {
            new_value1 += distance_value.at(m).second/distance_value.at(m).first;
            new_value2 += 1/distance_value.at(m).first;
          }
          int new_value = int(new_value1/new_value2);
          interpol_map.map_[j][k] = new_value;
        }
      }
    }


    nav_msgs::OccupancyGrid pub_map = empty_map_;
    nav_msgs::OccupancyGrid pub_map2 = empty_map_;

    for(int j=0;j<interpol_map.map_.size(); j++)
    {
      for (int k=0;k<interpol_map.map_[j].size(); k++)
      {
        pub_map.data.at(j + (k*empty_map_.info.width)) = interpol_map.map_[j][k];
      }
    }

    for(int j=0;j<scaled_map.map_.size(); j++)
    {
      for (int k=0;k<scaled_map.map_[j].size(); k++)
      {
        pub_map2.data.at(j + (k*empty_map_.info.width)) = scaled_map.map_[j][k];
      }
    }



    i->second.pub_.publish(pub_map);
    i->second.pub2_.publish(pub_map2);

    /*
    grid_map::GridMap grid_map({"original", "filtered"});
    grid_map.setFrameId("map");
    grid_map.setGeometry(grid_map::Length(empty_map_.info.width, empty_map_.info.height), empty_map_.info.resolution);
    grid_map::GridMapRosConverter::fromOccupancyGrid(pub_map2, "original", grid_map);

    cv::Mat originalImage;
    grid_map::GridMapCvConverter::toImage<unsigned short, 1>(grid_map, "original", CV_8UC1, 0.0, 255.0, originalImage);

    cv::Mat newImage;
    cv::GaussianBlur(originalImage, newImage, cv::Size(5,5), 0);

    grid_map::GridMapCvConverter::addLayerFromImage(newImage, "blur", grid_map, 0.0, 255.0);

    // Create OpenCV window.
    // cv::namedWindow("OpenCV Demo");
    // cv::imshow("OpenCV Demo", originalImage);
    // cv::waitKey(40);
    number++;
     */
  }
  return true;
}

double Subscriber::distance(int cell0, int cell1)
{
  double x0 = cell0%empty_map_.info.width;
  double x1 = cell1%empty_map_.info.width;
  double y0 = int(cell0/empty_map_.info.width);
  double y1 = int(cell1/empty_map_.info.width);
  double d = sqrt(pow(x1-x0,2) + pow(y1-y0,2));
  return d;
}

void Subscriber::insert_grid_distance_value(std::vector<std::pair<double, int> > &distance_value, int x1, int y1, int x2, int y2, int value)
{
  if(value != -1)
  {
    double distance = pow(sqrt(pow(x2 - x1,2) + pow(y2 - y1, 2)),2);
    distance_value.push_back(std::make_pair(distance, value));
  }
}