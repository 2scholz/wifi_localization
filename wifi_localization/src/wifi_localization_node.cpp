//
// Created by 2scholz on 12.08.16.
//
#include <ros/init.h>
#include <ros/node_handle.h>
#include "gaussian_process/gaussian_process.h"
#include "csv_data_loader.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/filesystem.hpp>
#include <std_srvs/Empty.h>
#include <wifi_localization/WifiState.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <fstream>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

class WifiLocalization
{
public:
  WifiLocalization(ros::NodeHandle &n)
  {
    std::string path = "";
    n_particles = 100;
    double p1_x = 0.0;
    double p1_y = 0.0;
    double p2_x = 0.0;
    double p2_y = 0.0;
    double p3_x = 0.0;
    double p3_y = 0.0;
    computing = false;

    n.param("/wifi_localization/path_to_csv", path, path);
    n.param("/wifi_localization/n_particles", n_particles, n_particles);
    n.param("/wifi_localization/p1_x", p1_x, p1_x);
    n.param("/wifi_localization/p1_y", p1_y, p1_y);
    n.param("/wifi_localization/p2_x", p2_x, p2_x);
    n.param("/wifi_localization/p2_y", p2_y, p2_y);
    n.param("/wifi_localization/p3_x", p3_x, p3_x);
    n.param("/wifi_localization/p3_y", p3_y, p3_y);

    Matrix<double, 3, 1> starting_point;
    starting_point = {2.3, 2.3, 2.65};

    bool existing_params = true;
    boost::filesystem::path param_path(path+"/parameters");

    if (!(boost::filesystem::exists(param_path)))
    {
      existing_params = false;
      boost::filesystem::create_directory(param_path);
    }

    for(directory_iterator itr(path); itr!=directory_iterator(); ++itr)
    {
      if(is_regular_file(itr->status()))
      {
        std::string file_path = path+"/"+itr->path().filename().generic_string();
        std::string mac = file_path.substr( file_path.find_last_of("/") + 1 );
        mac = mac.substr(0, mac.find_last_of("."));
        CSVDataLoader data(file_path);
        Process gp(data.points, data.observations);

        if(!existing_params)
        {
          gp.train_params(starting_point);

          boost::shared_ptr<std::ofstream> new_params = boost::make_shared<std::ofstream>();
          new_params->open(std::string(path+"/parameters/"+mac+".csv").c_str());
          *new_params << "signal_noise, signal_var, lengthscale" << "\n";
          Eigen::Vector3d parameters = gp.get_params();
          *new_params << std::to_string(parameters(0))+", "+std::to_string(parameters(1))+", "+std::to_string(parameters(2)) << "\n";
          new_params->flush();
        }
        else
        {
          std::ifstream file(path+"/parameters/"+mac+".csv");
          std::string value;
          std::string signal_noise;
          std::string signal_var;
          std::string lengthscale;
          getline(file, value, '\n');
          getline(file, signal_noise, ',');
          getline(file, signal_var, ',');
          getline(file, lengthscale, '\n');
          std::cout << signal_noise << signal_var << lengthscale << std::endl;
          gp.set_params(std::stod(signal_noise), std::stod(signal_var), std::stod(lengthscale));
        }

        std::cout << "loaded params: " << gp.get_params() << std::endl;

        gps.insert(gps.begin(), std::make_pair(mac, gp));
        //ROS_INFO(("Finished reading file: " + file_path).c_str());
      }
    }

    if(p1_x == 0.0 && p1_y == 0.0 && p2_x == 0.0 && p2_y == 0.0 && p3_x == 0.0 && p3_y == 0.0)
    {
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
      }
      p1_x = amcl_map_.info.origin.position.x;
      p1_y = amcl_map_.info.origin.position.y + double(amcl_map_.info.height) * amcl_map_.info.resolution;
      p2_x = p1_x + amcl_map_.info.width * amcl_map_.info.resolution;
      p2_y = p1_y;
      p3_x = p1_x;
      p3_y = amcl_map_.info.origin.position.y;
    }

    A = {p1_x, p1_y};
    Eigen::Vector2d B = {p2_x, p2_y};
    Eigen::Vector2d C = {p3_x, p3_y};
    AB = B - A;
    AC = C - A;

    compute_starting_point_service = n.advertiseService("compute_amcl_start_point", &WifiLocalization::compute_pose, this);
    initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
    sub = n.subscribe("wifi_state", 1000, &WifiLocalization::wifi_callback, this);
    ROS_INFO("Finished initialization.");
  }

  Eigen::Vector2d random_position()
  {
    double u = (double)rand() / RAND_MAX;
    double v = (double)rand() / RAND_MAX;

    return (A + u*AB + v*AC);
  }

private:
  Eigen::Vector2d A;
  Eigen::Vector2d AB;
  Eigen::Vector2d AC;
  bool computing;
  std::map<std::string, Process> gps;
  int n_particles;
  ros::Publisher initialpose_pub;
  ros::Subscriber sub;
  ros::ServiceServer compute_starting_point_service;
  std::vector<std::pair<std::string, double>> macs_and_strengths;

  bool compute_pose(std_srvs::Empty::Request  &req,
                    std_srvs::Empty::Response &res)
  {
    computing = true;
    Vector2d most_likely_pos;
    double highest_prob = 0.0;

    for(int i = 0; i < n_particles; ++i)
    {
      double total_prob = 1.0;

      Eigen::Vector2d random_point = random_position();
      for(auto it:macs_and_strengths)
      {
        std::map<std::string, Process>::iterator data = gps.find(it.first);

        if(data != gps.end())
        {
          double prob = data->second.probability(random_point(0), random_point(1), it.second);
          total_prob *= prob;
        }
      }
      if(total_prob > highest_prob)
      {
        highest_prob = total_prob;
        most_likely_pos = {random_point(0), random_point(1)};
        std::cout << "Newest most likely pos: " << random_point(0) << " and " << random_point(1) << std::endl;
        std::cout << "With probability: " << highest_prob << std::endl;
      }
    }

    computing = false;

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = most_likely_pos(0);
    pose.pose.pose.position.y = most_likely_pos(1);
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation.w = 1.0;

    initialpose_pub.publish(pose);

    return true;
  }

  void wifi_callback(const wifi_localization::WifiState::ConstPtr& msg)
  {
    if(!computing && !msg->macs.empty())
    {
      std::vector<std::pair<std::string, double>> mas;
      for (int i = 0; i < msg->macs.size(); i++)
      {
        std::string mac_name = msg->macs.at(i);
        double wifi_dbm = msg->strengths.at(i);

        mas.push_back(std::make_pair(mac_name, wifi_dbm));
      }
      macs_and_strengths = mas;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_localization");
  ros::NodeHandle n;
  WifiLocalization wl(n);

  ros::spin();

  return 0;
}