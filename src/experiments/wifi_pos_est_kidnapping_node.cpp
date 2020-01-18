//
// Created by scholz on 03.10.16.
//

#include <ros/init.h>
#include <ros/node_handle.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Duration.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <wifi_localization/MaxWeight.h>

class SubscriptionHandler
{
public:
  SubscriptionHandler(ros::NodeHandle &n) : n_(n)
  {
    old_pose_sub = n.subscribe("amcl_pose_old", 1000, &SubscriptionHandler::old_pose_callback, this);
    new_pose_sub = n.subscribe("amcl_pose", 1000, &SubscriptionHandler::new_pose_callback, this);
    goal_counter_sub = n.subscribe("goal_count", 1000, &SubscriptionHandler::goal_counter_callback, this);
    max_weight_sub = n.subscribe("max_weight", 1000, &SubscriptionHandler::max_weight_callback, this);

    diff_pub = n.advertise<std_msgs::Float64>("pose_diff", 1000);
    diff_at_goal_pub = n.advertise<std_msgs::Float64>("pose_diff_at_goal", 1000);
    initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
    seconds_to_convergence_pub = n.advertise<std_msgs::Duration>("seconds_to_convergence", 1000);

    wifi_pos_estimation_client = n.serviceClient<std_srvs::Empty>("compute_amcl_start_point");
    global_localization_client = n.serviceClient<std_srvs::Empty>("global_localization");

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

    A = {amcl_map_.info.origin.position.x, amcl_map_.info.origin.position.y + amcl_map_.info.height * amcl_map_.info.resolution};
    Eigen::Vector2d B = {A[0] + amcl_map_.info.width * amcl_map_.info.resolution, A[1]};
    Eigen::Vector2d C = {A[0], amcl_map_.info.origin.position.y};
    AB = B - A;
    AC = C - A;

    converged = true;
    pos_est_at_next_goal = false;
    experiment_running = false;
    ROS_INFO("Done initializing");
  }
private:
  ros::NodeHandle &n_;

  ros::Subscriber old_pose_sub;
  ros::Subscriber new_pose_sub;
  ros::Subscriber goal_counter_sub;
  ros::Subscriber max_weight_sub;

  ros::Publisher diff_pub;
  ros::Publisher diff_at_goal_pub;
  ros::Publisher seconds_to_convergence_pub;
  ros::Publisher initialpose_pub;

  ros::ServiceClient wifi_pos_estimation_client;
  ros::ServiceClient global_localization_client;

  geometry_msgs::PoseWithCovarianceStamped old_pose;
  geometry_msgs::PoseWithCovarianceStamped new_pose;

  Eigen::Vector2d A;
  Eigen::Vector2d AB;
  Eigen::Vector2d AC;

  bool converged;
  bool pos_est_at_next_goal;
  bool experiment_running;
  ros::Time kidnap_start_time;

  void old_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    old_pose = *msg;
  }

  void new_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    new_pose = *msg;
    std_msgs::Float64 diff_msg;
    diff_msg.data = sqrt(pow((new_pose.pose.pose.position.x - old_pose.pose.pose.position.x),2)+pow((new_pose.pose.pose.position.y - old_pose.pose.pose.position.y),2));

    diff_pub.publish(diff_msg);
    if(diff_msg.data < 0.2 && experiment_running)
    {
      converged = true;
      std_msgs::Duration time_to_convergence_msg;
      time_to_convergence_msg.data = kidnap_start_time - ros::Time::now();
      seconds_to_convergence_pub.publish(time_to_convergence_msg);
      experiment_running = false;
    }
  }

  void goal_counter_callback(const std_msgs::Int32::ConstPtr& msg)
  {
    std_msgs::Float64 diff_msg;
    diff_msg.data = sqrt(pow((new_pose.pose.pose.position.x - old_pose.pose.pose.position.x),2)+pow((new_pose.pose.pose.position.y - old_pose.pose.pose.position.y),2));

    diff_at_goal_pub.publish(diff_msg);
    if(pos_est_at_next_goal)
    {
      std_srvs::Empty srv_msg;

      if (wifi_pos_estimation_client.call(srv_msg))
      {
        ROS_INFO("Published position estimation");
      }
      else
      {
        ROS_INFO("Failed to publish position estimation");
      }
      pos_est_at_next_goal = false;
    }

    if(msg->data == 1)
    {
      initialpose_pub.publish(old_pose);
    }

    if(msg->data==3)
    {
      Eigen::Vector2d random_pos = random_position();

      geometry_msgs::PoseWithCovarianceStamped pose;
      pose.header.frame_id = "map";
      pose.pose.pose.position.x = random_pos(0);
      pose.pose.pose.position.y = random_pos(1);
      pose.pose.pose.position.z = 0.0;
      pose.pose.pose.orientation.w = 1.0;
      initialpose_pub.publish(pose);
      converged = false;
      kidnap_start_time = ros::Time::now();
      experiment_running = true;
    }
  }

  Eigen::Vector2d random_position()
  {
    double u = (double)rand() / RAND_MAX;
    double v = (double)rand() / RAND_MAX;

    return (A + u*AB + v*AC);
  }
  void max_weight_callback(const wifi_localization::MaxWeight::ConstPtr& msg)
  {
    if(msg->max_weight > 0.1)
    {
      pos_est_at_next_goal = true;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kidnapping_experiment");
  ros::NodeHandle n;

  SubscriptionHandler sh(n);
  ros::spin();

  return 0;
}
