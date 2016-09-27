#include <ros/init.h>
#include <ros/node_handle.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

void goal_count_callback(const std_msgs::Int32::ConstPtr& msg)
{
  std_srvs::Empty srv_msg;
  if (ros::service::call("wifi_position_estimation", srv_msg))
  {
    ROS_INFO("Published position estimation");
  }
  else
  {
    ROS_INFO("Failed to publish position estimation");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "accuracy_experiment");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("goal_count", 1000, goal_count_callback);

  ros::spin();

  return 0;

}
