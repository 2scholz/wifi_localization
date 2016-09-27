#include <ros/init.h>
#include <ros/node_handle.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class SubscriptionHandler
{
public:
  SubscriptionHandler(ros::NodeHandle &n) : n_(n)
  {
    old_pose_sub = n.subscribe("amcl_pose_old", 1000, &SubscriptionHandler::old_pose_callback, this);
    new_pose_sub = n.subscribe("amcl_pose", 1000, &SubscriptionHandler::new_pose_callback, this);
    goal_counter_sub = n.subscribe("goal_count", 1000, &SubscriptionHandler::goal_counter_callback, this);

    diff_pub = n.advertise<std_msgs::Float64>("pose_diff", 1000);
    diff_at_goal_pub = n.advertise<std_msgs::Float64>("pose_diff_at_goal", 1000);

    wifi_pos_estimation_client = n.serviceClient<std_srvs::Empty>("compute_amcl_start_point");
    global_localization_client = n.serviceClient<std_srvs::Empty>("global_localization");
  }
private:
  ros::NodeHandle &n_;

  ros::Subscriber old_pose_sub;
  ros::Subscriber new_pose_sub;
  ros::Subscriber goal_counter_sub;

  ros::Publisher diff_pub;
  ros::Publisher diff_at_goal_pub;

  ros::ServiceClient wifi_pos_estimation_client;
  ros::ServiceClient global_localization_client;

  geometry_msgs::PoseWithCovarianceStamped old_pose;
  geometry_msgs::PoseWithCovarianceStamped new_pose;

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
  }

  void goal_counter_callback(const std_msgs::Int32::ConstPtr& msg)
  {
    std_msgs::Float64 diff_msg;
    diff_msg.data = sqrt(pow((new_pose.pose.pose.position.x - old_pose.pose.pose.position.x),2)+pow((new_pose.pose.pose.position.y - old_pose.pose.pose.position.y),2));

    diff_at_goal_pub.publish(diff_msg);

    std_srvs::Empty srv_msg;
    if (wifi_pos_estimation_client.call(srv_msg))
    {
      ROS_INFO("Published position estimation");
    }
    else
    {
      ROS_INFO("Failed to publish position estimation");
    }
    /*
    if (global_localization_client.call(srv_msg))
    {
      ROS_INFO("Called global_localization");
    }
    else
    {
      ROS_INFO("Failed to call global_localization");
    }
    */
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "accuracy_experiment");
  ros::NodeHandle n;

  SubscriptionHandler sh(n);
  ros::spin();

  return 0;
}