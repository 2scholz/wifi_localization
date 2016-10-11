#include "wifi_position_estimation/wifi_position_estimation.h"

WifiPositionEstimation::WifiPositionEstimation(ros::NodeHandle &n)
{
  std::string path = "";
  n_particles_ = 100;
  computing_ = false;

  n.param("/wifi_position_estimation/path_to_csv", path, path);
  n.param("/wifi_position_estimation/n_particles", n_particles_, n_particles_);
  n.param("/wifi_position_estimation/quality_threshold", quality_threshold_, quality_threshold_);

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
      Process gp(data.points_, data.observations_);

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

      gp_map_.insert(gp_map_.begin(), std::make_pair(mac, gp));
      //ROS_INFO(("Finished reading file: " + file_path).c_str());
    }
  }

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

  A_ = {amcl_map_.info.origin.position.x, amcl_map_.info.origin.position.y + amcl_map_.info.height * amcl_map_.info.resolution};
  Eigen::Vector2d B = {A_[0] + amcl_map_.info.width * amcl_map_.info.resolution, A_[1]};
  Eigen::Vector2d C = {A_[0], amcl_map_.info.origin.position.y};
  AB_ = B - A_;
  AC_ = C - A_;

  compute_starting_point_service_ = n.advertiseService("compute_amcl_start_point", &WifiPositionEstimation::publish_pose_service, this);
  publish_accuracy_data_service_ = n.advertiseService("wifi_position_estimation", &WifiPositionEstimation::publish_accuracy_data, this);
  initialpose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
  wifi_sub_ = n.subscribe("wifi_state", 1000, &WifiPositionEstimation::wifi_callback, this);
  max_weight_sub_ = n.subscribe("max_weight", 1000, &WifiPositionEstimation::max_weight_callback, this);
  wifi_pos_estimation_pub_ = n.advertise<wifi_localization::WifiPositionEstimation>("wifi_pos_estimation_data", 1000);
  amcl_sub_ = n.subscribe("amcl_pose", 1000, &WifiPositionEstimation::amcl_callback, this);
  ROS_INFO("Finished initialization.");
}

Eigen::Vector2d WifiPositionEstimation::random_position()
{
  double u = (double)rand() / RAND_MAX;
  double v = (double)rand() / RAND_MAX;

  return (A_ + u*AB_ + v*AC_);
}

bool WifiPositionEstimation::publish_pose_service(std_srvs::Empty::Request  &req,
                          std_srvs::Empty::Response &res)
{
  geometry_msgs::PoseWithCovarianceStamped pose = compute_pose();
  pose.header.stamp = ros::Time::now();
  initialpose_pub_.publish(pose);
  computing_ = false;

  return true;
}

bool WifiPositionEstimation::publish_accuracy_data(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  geometry_msgs::PoseWithCovarianceStamped pose = compute_pose();
  wifi_localization::WifiPositionEstimation msg;
  msg.pos_x = pose.pose.pose.position.x;
  msg.pos_y = pose.pose.pose.position.y;
  msg.amcl_diff = sqrt(pow((pose.pose.pose.position.x - x_pos_),2)+pow((pose.pose.pose.position.y - y_pos_),2));
  msg.header.stamp = ros::Time::now();
  wifi_pos_estimation_pub_.publish(msg);
  computing_ = false;

  return true;
}

geometry_msgs::PoseWithCovarianceStamped WifiPositionEstimation::compute_pose()
{
  computing_ = true;
  Vector2d most_likely_pos;
  double highest_prob = 0.0;

  for(int i = 0; i < n_particles_; ++i)
  {
    double total_prob = 1.0;

    Eigen::Vector2d random_point = random_position();
    for(auto it:macs_and_strengths_)
    {
      std::map<std::string, Process>::iterator data = gp_map_.find(it.first);

      if(data != gp_map_.end())
      {
        double prob = data->second.probability(random_point(0), random_point(1), it.second);
        if(isnan(prob))
          prob = 1.0;
        total_prob *= prob;
      }
    }
    std::cout << "total_prob: " << total_prob << std::endl;
    if(total_prob > highest_prob)
    {
      highest_prob = total_prob;
      most_likely_pos = {random_point(0), random_point(1)};
      std::cout << "Newest most likely pos: " << random_point(0) << " and " << random_point(1) << std::endl;
      std::cout << "With probability: " << highest_prob << std::endl;
    }
  }

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.pose.pose.position.x = most_likely_pos(0);
  pose.pose.pose.position.y = most_likely_pos(1);
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance = {0.0};
  pose.pose.covariance[0] = 2.0;
  pose.pose.covariance[7] = 2.0;
  pose.pose.covariance[14] = 0.0;
  pose.pose.covariance[21] = 0.0;
  pose.pose.covariance[28] = 0.0;
  pose.pose.covariance[35] = M_PI;

  return pose;

}

void WifiPositionEstimation::wifi_callback(const wifi_localization::WifiState::ConstPtr& msg)
{
  if(!computing_ && !msg->macs.empty())
  {
    std::vector<std::pair<std::string, double>> mas;
    for (int i = 0; i < msg->macs.size(); i++)
    {
      std::string mac_name = msg->macs.at(i);
      double wifi_dbm = msg->strengths.at(i);

      mas.push_back(std::make_pair(mac_name, wifi_dbm));
    }
    macs_and_strengths_ = mas;
  }
}

void WifiPositionEstimation::max_weight_callback(const wifi_localization::MaxWeight::ConstPtr& msg)
{
  if(msg->max_weight > quality_threshold_ && !computing_)
  {
    compute_pose();
  }
}

void WifiPositionEstimation::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  x_pos_ = msg->pose.pose.position.x;
  y_pos_ = msg->pose.pose.position.y;
}