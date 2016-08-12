//
// Created by 2scholz on 10.08.16.
//

#include <ros/node_handle.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>

Eigen::Vector2d random_position(const Eigen::Vector2d &A, const Eigen::Vector2d &AB, const Eigen::Vector2d &AC)
{
  double u = (double)rand() / RAND_MAX;
  double v = (double)rand() / RAND_MAX;

  return (A + u*AB + v*AC);
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> step_vector(const Eigen::Vector2d &AB, const Eigen::Vector2d &AC, double step_length)
{
  Eigen::Vector2d step_right = (sqrt(step_length)/(pow(AB(0),2)+pow(AB(1),2))) * AB;
  Eigen::Vector2d step_down = (sqrt(step_length)/(pow(AC(0),2)+pow(AC(1),2))) * AC;
  return std::make_pair(step_right, step_down);
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

  bool ordered = false;
  double step_size = 0.20;

  n.param("/map_traverser/p1_x", p1_x, p1_x);
  n.param("/map_traverser/p1_y", p1_y, p1_y);
  n.param("/map_traverser/p2_x", p2_x, p2_x);
  n.param("/map_traverser/p2_y", p2_y, p2_y);
  n.param("/map_traverser/p3_x", p3_x, p3_x);
  n.param("/map_traverser/p3_y", p3_y, p3_y);
  n.param("/map_traverser/ordered", ordered, ordered);
  n.param("/map_traverser/step_size", step_size, step_size);

  if(p1_x == 0.0 && p1_y == 0.0 && p2_x == 0.0 && p2_y == 0.0 && p3_x == 0.0 && p3_y == 0.0)
  {
    ROS_ERROR("Please provide three points that span a square, with the first being the upper left corner, the second "
                "being the upper right corner and the third being the lower left corner.");
    return 0;
  }

  Eigen::Vector2d A = {p1_x, p1_y};
  Eigen::Vector2d B = {p2_x, p2_y};
  Eigen::Vector2d C = {p3_x, p3_y};
  Eigen::Vector2d AB = B - A;
  Eigen::Vector2d AC = C - A;

  Eigen::Vector2d unit1 = {1.0, 0.0};
  Eigen::Vector2d unit2 = {0.0, -1.0};

  Eigen::Vector2d A2 = {0.0, 0.0};
  Eigen::Vector2d AB2 = AB.norm()*unit1;
  Eigen::Vector2d AC2 = AC.norm()*unit2;

  MoveBaseClient ac("move_base", true);

  ros::Rate r(10);
  DataRecorded data_recorded;
  ros::Subscriber data_recorded_sub = n.subscribe("recorded_since_stop", 1000, &DataRecorded::callback, &data_recorded);

  if(ordered)
  {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> step_vectors = step_vector(AB, AC, step_size);
    Eigen::Vector2d step_right = step_vectors.first;
    Eigen::Vector2d step_down = step_vectors.second;


    double step_right_angle = fmod(atan2(step_right(1), step_right(0)), (2.0*M_PI));
    double step_down_angle = fmod(atan2(step_down(1), step_down(0)), (2.0*M_PI));

    double angle_z = 0.0;
    double angle_w = 0.0;

    if(AB.norm() > AC.norm())
    {
      angle_z = sin(step_right_angle/2.0);
      angle_w = cos(step_right_angle/2.0);
    }
    else
    {
      angle_z = sin(step_down_angle/2.0);
      angle_w = cos(step_down_angle/2.0);
    }

    // First drive to the start point, which is the upper left corner of the square.
    double pos_x = p1_x;
    double pos_y = p1_y;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pos_x;
    goal.target_pose.pose.position.y = pos_y;
    goal.target_pose.pose.orientation.z = angle_z;
    goal.target_pose.pose.orientation.w = angle_w;

    ROS_INFO("Reached.");

    Eigen::Vector2d pos = A;
    Eigen::Vector2d check_pos = A2;
    Eigen::Vector2d check_step_right = {step_size, 0.0};
    Eigen::Vector2d check_step_down = {0.0, -step_size};

    int sign = 1;

    while(ros::ok())
    {
      if(data_recorded.data_recorded)
      {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = pos(0);
        goal.target_pose.pose.position.y = pos(1);
        goal.target_pose.pose.orientation.z = sign * angle_z;
        goal.target_pose.pose.orientation.w = angle_w;
        ac.sendGoal(goal);
        ac.waitForResult();

        if(AB.norm() > AC.norm())
        {
          if((check_pos(0) + sign * check_step_right(0) > AB2(0))||(check_pos(1) + sign * check_step_right(1) < AC(1)))
          {
            if((check_pos(0) + check_step_down(0) > AB2(0))||(check_pos(1) + check_step_down(1) < AC(1)))
            {
              ROS_INFO("Map was fully traversed.");
              return 0;
            }
            pos = pos + step_down;
            check_pos = check_pos + check_step_down;
            sign = -1 * sign;
          }
          else
          {
            pos = pos + sign * step_right;
            check_pos = check_pos + sign * check_step_right;
          }
        }
        else
        {
          if((check_pos(0) + sign * check_step_down(0) > AB2(0))||(check_pos(1) + sign * check_step_down(1) < AC(1)))
          {
            if((check_pos(0) + check_step_right(0) > AB2(0))||(check_pos(1) + check_step_right(1) < AC(1)))
            {
              ROS_INFO("Map was fully traversed.");
              return 0;
            }
            pos = pos + step_right;
            check_pos = check_pos + check_step_right;
            sign = -1 * sign;
          }
          else
          {
            pos = pos + sign * step_down;
            check_pos = check_pos + sign * check_step_down;
          }
        }
      }
      ros::spinOnce();
      r.sleep();
    }
  }
  else
  {
    while(ros::ok())
    {
      if(data_recorded.data_recorded)
      {
        Eigen::Vector2d goal_point = random_position(A, AB, AC);
        double pos_x = goal_point(0);
        double pos_y = goal_point(1);
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
  }

  return 0;
}