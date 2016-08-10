//
// Created by 2scholz on 10.08.16.
//

#include <ros/node_handle.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>

std::pair<double, double> fRand(double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y)
{
  double f = (double)rand() / RAND_MAX;
  double u = 0.0 + f * (1.0 - 0.0);
  f = (double)rand() / RAND_MAX;
  double v = 0.0 + f * (1.0 - 0.0);

  double p_x = p1_x + u*(p2_x - p1_x) + v*(p3_x - p1_x);
  double p_y = p1_y + u*(p2_y - p1_y) + v*(p3_y - p1_y);
  return std::make_pair(p_x, p_y);
}

class DataRecorded
{
public:
  DataRecorded():data_recorded(false)
  {}
  void callback(const std_msgs::Bool::ConstPtr& msg)
  {
    data_recorded = msg->data;
  }
public:
  bool data_recorded;
};


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_traverser");
  ros::NodeHandle n;

  double p1_x = 0.0;
  double p1_y = 0.0;
  double p2_x = 0.0;
  double p2_y = 0.0;
  double p3_x = 0.0;
  double p3_y = 0.0;

  n.param("/map_traverser/p1_x", p1_x, p1_x);
  n.param("/map_traverser/p1_y", p1_y, p1_y);
  n.param("/map_traverser/p2_x", p2_x, p2_x);
  n.param("/map_traverser/p2_y", p2_y, p2_y);
  n.param("/map_traverser/p3_x", p3_x, p3_x);
  n.param("/map_traverser/p3_y", p3_y, p3_y);

  if(p1_x == 0.0 && p1_y == 0.0 && p2_x == 0.0 && p2_y == 0.0 && p3_x == 0.0 && p3_y == 0.0)
  {
    ROS_ERROR("Please provide three points that span a square, with the first being the upper left corner, the second "
                "being the upper right corner and the third being the lower left corner.");
    return 0;
  }

  MoveBaseClient ac("move_base", true);

  ros::Rate r(10);
  DataRecorded data_recorded;
  ros::Subscriber data_recorded_sub = n.subscribe("recorded_since_stop", 1000, &DataRecorded::callback, &data_recorded);

  while(ros::ok())
  {
    if(data_recorded.data_recorded)
    {
      std::pair<double, double> goal_point = fRand(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y);
      double pos_x = goal_point.first;
      double pos_y = goal_point.second;
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "/map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = pos_x;
      goal.target_pose.pose.position.y = pos_y;
      goal.target_pose.pose.orientation.w = 1.0;
      ac.sendGoal(goal);

      ac.waitForResult();
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}