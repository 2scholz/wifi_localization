//
// Created by 2scholz on 10.08.16.
//

#include <ros/node_handle.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Subscriber class for the odometry.
 */
class OdometrySub
{
public:
  OdometrySub(ros::NodeHandle &n) : n_(n)
  {
    odom_sub_ = n_.subscribe("/odom", 1, &OdometrySub::odometry_callback, this);
  }
  nav_msgs::Odometry odometry_;

private:
  ros::NodeHandle &n_;
  ros::Subscriber odom_sub_;
  void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    odometry_ = *msg;
  }
};

/**
 * Node to traverse the map. Given a vector of x- and y-coordinates as parameters, it drives to those coordinates one
 * after the other.
 * @param argc
 * @param argv
 * @return
 */
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
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

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

  OdometrySub os(n);

  for(unsigned int i = 0; i < x_positions.size(); i++)
  {
    double pos_x = x_positions.at(i);
    double pos_y = y_positions.at(i);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pos_x;
    goal.target_pose.pose.position.y = pos_y;
    //std::cout << pos_x << " " << pos_y << std::endl;
    ROS_INFO("Driving to position %f, %f", pos_x, pos_y);
    ac.sendGoalAndWait(goal);
    ROS_INFO("Reached goal");
    //std::cout << "reached" << std::endl;
    ++counter_msg.data;
    counter_pub.publish(counter_msg);
    ros::spinOnce();
    ros::Duration(10.0).sleep();
    geometry_msgs::Twist turn;
    turn.angular.z = 1.0;
    ros::Rate r(100); // 100 hz
    double start_yaw = tf::getYaw(os.odometry_.pose.pose.orientation);
    double halfway_yaw = start_yaw + M_PI;
    bool half_turn = false;
    bool full_turn = false;

    while(!half_turn || !full_turn)
    {
      vel_pub.publish(turn);
      double yaw = tf::getYaw(os.odometry_.pose.pose.orientation);

      if(!half_turn)
      {
        double diff = atan2(sin(halfway_yaw-yaw), cos(halfway_yaw-yaw));
        half_turn = diff < 0.1 && diff > -0.1;
      }
      else
      {
        double diff = atan2(sin(start_yaw-yaw), cos(start_yaw-yaw));
        full_turn = diff < 0.1 && diff > -0.1;
      }
      ros::spinOnce();
      r.sleep();
    }
    turn.angular.z = 0.0;
    vel_pub.publish(turn);
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Reached last goal.");

  return 0;
}