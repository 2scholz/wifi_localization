//
// Created by 2scholz on 10.08.16.
//

#include <ros/node_handle.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Core>

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

std::pair<double, double> step_vector(double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y,
                                      double step_length)
{
  double step_right = sqrt(step_length)/(pow((p2_x-p1_x),2)+pow(p2_y - p1_y,2));
  double step_down = sqrt(step_length)/(pow((p3_x-p1_x),2)+pow(p3_y - p1_y,2));
  return std::make_pair(step_right, step_down);
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> step_vector(Eigen::Vector2d AB, Eigen::Vector2d AC, double step_length)
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

  MoveBaseClient ac("move_base", true);

  ros::Rate r(10);
  DataRecorded data_recorded;
  ros::Subscriber data_recorded_sub = n.subscribe("recorded_since_stop", 1000, &DataRecorded::callback, &data_recorded);

  if(ordered)
  {
    /*
    std::pair<double, double> steps = step_vector(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, 0.20);
    double step_right_x = steps.first * (p2_x - p1_x);
    double step_right_y = steps.first * (p2_y - p1_y);
    double step_down_x = steps.second * (p3_x - p1_x);
    double step_down_y = steps.second * (p3_y - p1_y);
     */
    std::pair<Eigen::Vector2d, Eigen::Vector2d> step_vectors = step_vector(AB, AC, 0.20);
    Eigen::Vector2d step_right = step_vectors.first;
    Eigen::Vector2d step_down = step_vectors.second;

    /*
    double step_right_angle = acos(step_right_x/sqrt(pow(step_right_x,2)+pow(step_right_y,2)));
    double step_down_angle = acos(step_right_x/sqrt(pow(step_down_x,2)+pow(step_down_y,2)));
    */
    double step_right_angle = acos(step_right(0)/step_right.norm());
    double step_down_angle = acos(step_down(0)/step_down.norm());

    /*
    double vector_right_length = sqrt(pow((p2_x - p1_x),2) + pow((p2_y - p1_y),2));
    double vector_down_lenght = sqrt(pow((p3_x - p1_x),2) + pow((p3_y - p1_y),2));
    */
    double angle = 0.0;

    if(AB.norm() > AC.norm())
      angle = step_right_angle;
    else
      angle = step_down_angle;

    // First drive to the start point, which is the upper left corner of the square.
    double pos_x = p1_x;
    double pos_y = p1_y;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pos_x;
    goal.target_pose.pose.position.y = pos_y;
    goal.target_pose.pose.orientation.w = angle;
    ac.sendGoal(goal);
    ac.waitForResult();

    Eigen::Vector2d pos = A;

    int sign = 1;

    while(ros::ok())
    {
      if(data_recorded.data_recorded)
      {
        if(AB.norm() > AC.norm())
        {
          if(((pos(0) + step_right(0)) > (A(0) +AB(0) + AC(0))) || ((pos(1) + step_right(1)) > (A(1) +AB(1) + AC(1))))
          {
            if(((pos(0) + step_down(0)) > (A(0) +AB(0) + AC(0))) || ((pos(1) + step_down(1)) > (A(1) +AB(1) + AC(1))))
            {
              ROS_INFO("Map was fully traversed.");
              return 0;
            }
            pos = pos + step_down;
            sign = -1 * sign;
          }
          else
          {
            pos = pos + sign * step_right;
          }
        }
        else
        {
          if(((pos(0) + step_down(0)) > (A(0) +AB(0) + AC(0))) || ((pos(1) + step_down(1)) > (A(1) +AB(1) + AC(1))))
          {
            if(((pos(0) + step_right(0)) > (A(0) +AB(0) + AC(0))) || ((pos(1) + step_right(1)) > (A(1) +AB(1) + AC(1))))
            {
              ROS_INFO("Map was fully traversed.");
              return 0;
            }
            pos = pos + step_right;
            sign = -1 * sign;
          }
          else
          {
            pos = pos + sign * step_down;
          }
        }

        /*
        if(vector_right_length > vector_down_lenght)
        {
          if((pos_x + step_right_x) > (p1_x +(p2_x - p1_x) + (p3_x - p1_x)) || (pos_y+step_right_y) > (p1_y+(p2_y - p1_y)+(p3_y - p1_y)))
          {
            if((pos_x + step_down_x) > (p1_x +(p2_x - p1_x) + (p3_x - p1_x)) || (pos_y+step_down_y) > (p1_y+(p2_y - p1_y)+(p3_y - p1_y)))
            {
              ROS_INFO("Map was fully traversed.");
              return 0;
            }
            pos_x = pos_x + step_down_x;
            pos_y = pos_y + step_down_y;
            sign = -1 * sign;
          }
          else
          {
            pos_x = pos_x + sign * step_right_x;
            pos_y = pos_y + sign * step_right_y;
          }
        }
        else
        {
          if((pos_x + step_down_x) > (p1_x +(p2_x - p1_x) + (p3_x - p1_x)) || (pos_y+step_down_y) > (p1_y+(p2_y - p1_y)+(p3_y - p1_y)))
          {
            if((pos_x + step_right_x) > (p1_x +(p2_x - p1_x) + (p3_x - p1_x)) || (pos_y+step_right_y) > (p1_y+(p2_y - p1_y)+(p3_y - p1_y)))
            {
              ROS_INFO("Map was fully traversed.");
              return 0;
            }
            pos_x = pos_x + step_down_x;
            pos_y = pos_y + step_down_y;
            sign = -1 * sign;
          }
          else
          {
            pos_x = pos_x + sign * step_down_x;
            pos_y = pos_y + sign * step_down_y;
          }
        }
        */
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = pos(0);
        goal.target_pose.pose.position.y = pos(1);
        goal.target_pose.pose.orientation.w = sign * angle;
        ac.sendGoal(goal);
        ac.waitForResult();
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
  }

  return 0;
}