#include <grid_map_ros/GridMapRosConverter.hpp>
#include "wifi_position_estimation/wifi_position_estimation.h"

WifiPositionEstimation::WifiPositionEstimation(ros::NodeHandle &n):precomputed_data_(cmp),
                                                                   gp_grid_map_({"gp_mean", "gp_variance"})
{
  std::string path = "";
  n_particles_ = 100;
  computing_ = false;
  precompute_ = true;

  init_noise_ = 2.3;
  init_var_ = 2.3;
  init_l1_ = -7.0;
  init_l2_ = -7.0;


  n.param("/wifi_position_estimation/path_to_csv", path, path);
  n.param("/wifi_position_estimation/n_particles", n_particles_, n_particles_);
  n.param("/wifi_position_estimation/quality_threshold", quality_threshold_, quality_threshold_);
  n.param("/wifi_position_estimation/precompute", precompute_, precompute_);
  n.param("/wifi_position_estimation/init_noise", init_noise_, init_noise_);
  n.param("/wifi_position_estimation/init_var", init_var_, init_var_);
  n.param("/wifi_position_estimation/init_l1", init_l1_, init_l1_);
  n.param("/wifi_position_estimation/init_l2", init_l2_, init_l2_);

  ROS_INFO("particle count: %i", n_particles_);
  if(!path.empty())
  {
    ROS_INFO("Loading all csv-files from path: %s", path.c_str());
  }
  else
  {
    ROS_ERROR("No path to csv-files provided.");
  }
  ROS_INFO("Threshold to trigger Wi-Fi position estimation: %f", quality_threshold_);
  ROS_INFO("Starting Initialization.");

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
    ROS_WARN("Failed to call map service. Using default map instead.");
    amcl_map_.info.map_load_time = ros::Time::now();
    amcl_map_.info.resolution = 0.20;
    amcl_map_.info.height = 1000;
    amcl_map_.info.width = 1000;

    amcl_map_.info.origin.position.x = 0.0;
    amcl_map_.info.origin.position.y = 0.0;
    amcl_map_.info.origin.position.z = 0.0;

    amcl_map_.info.origin.orientation.x = 0.0;
    amcl_map_.info.origin.orientation.y = 0.0;
    amcl_map_.info.origin.orientation.z = 0.0;
    amcl_map_.info.origin.orientation.w = 1.0;
    amcl_map_.data.resize(amcl_map_.info.width * amcl_map_.info.height);

  }

  A_ = {amcl_map_.info.origin.position.x, amcl_map_.info.origin.position.y + amcl_map_.info.height * amcl_map_.info.resolution};
  Eigen::Vector2d B = {A_[0] + amcl_map_.info.width * amcl_map_.info.resolution, A_[1]};
  Eigen::Vector2d C = {A_[0], amcl_map_.info.origin.position.y};
  AB_ = B - A_;
  AC_ = C - A_;

  if(precompute_)
  {
    for(int i=0;i<n_particles_;i++)
    {
      Eigen::Vector2d current_coordinate = random_position();
      random_points_.push_back(current_coordinate);
    }
  }

  Matrix<double, 4, 1> starting_point;

  starting_point = {init_noise_, init_var_, init_l1_, init_l2_};

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
      Process gp(data.coordinates_matrix_, data.observations_matrix_, 0.0, 0.0,  {0.0,0.0});

      if(!existing_params)
      {
        ROS_INFO("Training Gaussian process with data from path: %s", file_path.c_str());
        gp.train_params(starting_point);

        boost::shared_ptr<std::ofstream> new_params = boost::make_shared<std::ofstream>();
        new_params->open(std::string(path+"/parameters/"+mac+".csv").c_str());
        *new_params << "signal_noise, signal_var, lengthscale" << "\n";
        Eigen::Vector4d parameters = gp.get_params();
        *new_params << std::to_string(parameters(0))+", "+std::to_string(parameters(1))+", "+std::to_string(parameters(2))+", "+std::to_string(parameters(3)) << "\n";
        new_params->flush();
      }
      else
      {
        std::ifstream file(path+"/parameters/"+mac+".csv");
        std::string value;
        std::string signal_noise;
        std::string signal_var;
        std::string lengthscale;
        std::string lengthscale2;
        getline(file, value, '\n');
        getline(file, signal_noise, ',');
        getline(file, signal_var, ',');
        getline(file, lengthscale, ',');
        getline(file, lengthscale2, '\n');

        gp.set_params(std::stod(signal_noise), std::stod(signal_var), std::stod(lengthscale), std::stod(lengthscale2));
      }

      //std::cout << "loaded params: " << gp.get_params() << std::endl;


      std::replace(mac.begin(),mac.end(),'_',':');
      Eigen::Vector4d parameters = gp.get_params();

      if(parameters(0) != 0.0 || parameters(1) != 0.0 || parameters(2) != 0.0 || parameters(3) != 0.0)
      {
        auto current_gp_it = gp_map_.insert(gp_map_.begin(), std::make_pair(mac, gp));
        if(precompute_)
        {
          for(auto& random_position:random_points_)
          {
            PrecomputedDataPoint data_point{&current_gp_it->second, 0.0, 0.0};
            data_point.gp_->precompute_data(data_point, random_position);
            precomputed_data_[random_position][mac] = data_point;
          }
        }
      }
    }
  }

  gp_grid_map_.setFrameId("map");

  grid_map::GridMapRosConverter::fromOccupancyGrid(amcl_map_, "gp_mean", gp_grid_map_);
  grid_map::GridMapRosConverter::fromOccupancyGrid(amcl_map_, "gp_variance", gp_grid_map_);
  gp_grid_map_.setGeometry(gp_grid_map_.getLength(),1.0,gp_grid_map_.getPosition());

  grid_map_publisher_ = n.advertise<grid_map_msgs::GridMap>("grid_map", 1000, true);

  compute_starting_point_service_ = n.advertiseService("compute_amcl_start_point", &WifiPositionEstimation::publish_pose_service, this);
  publish_accuracy_data_service_ = n.advertiseService("wifi_position_estimation", &WifiPositionEstimation::publish_accuracy_data, this);
  publish_grid_map_service_ = n.advertiseService("create_map_of_gp", &WifiPositionEstimation::publish_gp_map_service, this);
  initialpose_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
  wifi_sub_ = n.subscribe("wifi_data", 1000, &WifiPositionEstimation::wifi_callback, this);
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
  msg.pos_x = x_pos_;
  msg.pos_y = y_pos_;
  msg.estimated_pos_x = pose.pose.pose.position.x;
  msg.estimated_pos_y = pose.pose.pose.position.y;
  msg.amcl_diff = sqrt(pow((pose.pose.pose.position.x - x_pos_),2)+pow((pose.pose.pose.position.y - y_pos_),2));
  msg.header.stamp = ros::Time::now();
  wifi_pos_estimation_pub_.publish(msg);
  computing_ = false;

  return true;
}

geometry_msgs::PoseWithCovarianceStamped WifiPositionEstimation::compute_pose()
{
  computing_ = true;
  ROS_INFO("Starting position estimation.");
  Vector2d most_likely_pos;
  double highest_prob = 0.0;
  std::sort(macs_and_strengths_.begin(), macs_and_strengths_.end(),
            boost::bind(&std::pair<std::string, double>::second, _1) >
            boost::bind(&std::pair<std::string, double>::second, _2));

  if(precompute_)
  {
    // Iterate over the coordinates
    for(auto& it:precomputed_data_)
    {
      Eigen::Vector2d current_coordinate = it.first;
      // std::cout << "current coordinate: " << current_coordinate(0) << ", " << current_coordinate(1) << std::endl;

      double total_prob = 1.0;

      // Iterate over the current macs and the associated strengths
      for(auto& it2:macs_and_strengths_)
      {
        auto data = it.second.find(it2.first);

        if(data != it.second.end())
        {
          double prob = data->second.gp_->probability_precomputed(data->second.mean_, data->second.variance_, it2.second);

          if(std::isnan(prob))
            prob = 1.0;
          total_prob *= prob;
        }
      }
      if(total_prob > highest_prob)
      {
        highest_prob = total_prob;
        most_likely_pos = {current_coordinate(0), current_coordinate(1)};
        // std::cout << "Newest most likely pos: " << random_point(0) << " and " << random_point(1) << std::endl;
        // std::cout << "With probability: " << highest_prob << std::endl;
      }
    }
  }

  else
  {
    for(int i = 0; i < n_particles_; ++i)
    {
      double total_prob = 1.0;

      Eigen::Vector2d random_point = random_position();

      // std::cout << "current coordinate: " << random_point(0) << ", " << random_point(1) << std::endl;

      for(auto it:macs_and_strengths_)
      {
        std::map<std::string, Process>::iterator data = gp_map_.find(it.first);

        if(data != gp_map_.end())
        {
          double prob = data->second.probability(random_point(0), random_point(1), it.second);
          if(std::isnan(prob))
            prob = 1.0;
          total_prob *= prob;
        }
      }
      if(total_prob > highest_prob)
      {
        highest_prob = total_prob;
        most_likely_pos = {random_point(0), random_point(1)};
        // std::cout << "Newest most likely pos: " << random_point(0) << " and " << random_point(1) << std::endl;
        // std::cout << "With probability: " << highest_prob << std::endl;
      }
    }
  }

  ROS_INFO("Estimated position: %f, %f", most_likely_pos(0), most_likely_pos(1));

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

bool WifiPositionEstimation::publish_gp_map_service(wifi_localization::PlotGP::Request &req,
                                                    wifi_localization::PlotGP::Response &res)
{
  std::string mac = req.mac;

  auto it = gp_map_.find(mac);
  if(it == gp_map_.end())
  {
    ROS_ERROR("The mac provided for the service for publishing the grid map of the gaussian process was not found.");
    return false;
  }

  ROS_INFO("Found mac. Begin to plot map.");
  it->second.create_gp_mean_map(gp_grid_map_);
  it->second.create_gp_variance_map(gp_grid_map_);

  ros::Time time = ros::Time::now();

  // Publish grid map.
  gp_grid_map_.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(gp_grid_map_, message);
  message.info.header.frame_id = "map";
  grid_map_publisher_.publish(message);
  ROS_INFO("Plot finished.");
  return true;
}