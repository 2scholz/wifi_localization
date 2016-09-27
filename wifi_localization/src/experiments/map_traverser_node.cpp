//
// Created by 2scholz on 10.08.16.
//

#include <ros/node_handle.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_traverser");
  ros::NodeHandle n;
  std::vector<double> x_positions;
  std::vector<double> y_positions;
  double angle_z;
  double angle_w;

  n.param("/map_traverser/x_positions", x_positions, x_positions);
  n.param("/map_traverser/y_positions", y_positions, y_positions);
  n.param("/map_traverser/angle_z", angle_z, angle_z);
  n.param("/map_traverser/angle_w", angle_w, angle_w);

  ros::Publisher counter_pub = n.advertise<std_msgs::Int32>("goal_count", 1000);

  if(x_positions.size() != y_positions.size())
  {
    ROS_ERROR("Please provide lists of x- and y-positions that are equal in length.");
    return 1;
  }


  ros::Rate r(10);
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.orientation.z = angle_z;
  goal.target_pose.pose.orientation.w = angle_w;
  goal.target_pose.header.frame_id = "/map";

  std_msgs::Int32 counter_msg;
  counter_msg.data = 0;

  std_srvs::Empty srv;

  for(unsigned int i = 0; i < x_positions.size(); i++)
  {
    double pos_x = x_positions.at(i);
    double pos_y = y_positions.at(i);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pos_x;
    goal.target_pose.pose.position.y = pos_y;
    std::cout << pos_x << " " << pos_y << std::endl;
    ac.sendGoalAndWait(goal);
    std::cout << "reached" << std::endl;
    ++counter_msg.data;
    counter_pub.publish(counter_msg);
    ros::spinOnce();
    ros::Duration(60.0).sleep();
  }

  return 0;
}