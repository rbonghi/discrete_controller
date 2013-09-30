/* 
 * File:   discrete_controller.cpp
 * Author: raffaello
 *
 * Created on 08 September 2013, 16:54
 */

#include <cstdlib>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transformation/Transform.h"
#include "PathPlotter.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <motion_control/Velocity.h>
#include <motion_control/Pose.h>
#include <discrete_controller/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

PathPlotter* path;
Transform alpha;
discrete_controller::Command cmd;
geometry_msgs::PoseStamped pose_robot;

const std::string name_node = "discrete";
const std::string gain_string = "gain";
double k1, k2, k3;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose_robot.header = msg.get()->header;
  const geometry_msgs::Pose pose_goal = msg.get()->pose;
  ROS_INFO("Robot [%f, %f, %f]", pose_robot.pose.position.x, pose_robot.pose.position.y, tf::getYaw(pose_robot.pose.orientation));
  ROS_INFO("Goal [%f, %f, %f]", pose_goal.position.x, pose_goal.position.y, tf::getYaw(pose_goal.orientation));
  path->setGoal(&pose_robot, msg.get(), (AbstractTransform*) new Transform());
  path->startController(&pose_robot, msg.get(), (AbstractTransform*) new Transform(), "robot", "odometry", "command/velocity");
}

void odometry_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_robot.pose = msg.get()->pose.pose;
}

motion_control::Velocity path_controller(ros::NodeHandle nh, motion_control::Velocity velocityd, geometry_msgs::PoseStamped posed, nav_msgs::Odometry pose_robot)
{
  motion_control::Velocity velocity;
  double theta = tf::getYaw(pose_robot.pose.pose.orientation);
  double error_x = posed.pose.position.x - pose_robot.pose.pose.position.x;
  double error_y = posed.pose.position.y - pose_robot.pose.pose.position.y;
  double error_th = tf::getYaw(posed.pose.orientation) - theta;

  double e1 = cos(theta) * error_x + sin(theta) * error_y;
  double e2 = cos(theta) * error_y - sin(theta) * error_x;
  double e3 = error_th;

  double u1 = -k1*e1;
  double u2 = -k2 * e2 - k3*e3;

  velocity.lin_vel = velocityd.lin_vel * cos(e3) - u1;
  velocity.ang_vel = velocityd.ang_vel - u2;
  return velocity;
}

discrete_controller::Command rate_fnc(int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal)
{
  Transform zr(pose_robot);
  //  ROS_INFO("zr [%f, %f, %f]", zr.state.z1, zr.state.z2, zr.state.z3);
  Transform zk(pose_goal);
  //  ROS_INFO("zk [z1: %f, z2: %f, z3: %f]", zk.state.z1, zk.state.z2, zk.state.z3);
  alpha = (zk - zr) / delta;
  //  ROS_INFO("alpha [%f, %f, %f]", alpha.state.z1, alpha.state.z2, alpha.state.z3);

  cmd.u1 = alpha.state.z1;
  cmd.u2 = (-alpha.state.z2 + (4 * alpha.state.z3) / (delta * alpha.state.z1) - (4 * zr.state.z2) / (delta));

  //  ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
  return cmd;
}

discrete_controller::Command multirate_fnc(int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal)
{

  double u21 = cmd.u2;
  cmd.u2 = (2 * alpha.state.z2 - u21);

  //  ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
  return cmd;
}

/*
 * 
 */
int main(int argc, char** argv)
{

  //Init the serial_motion_node
  ros::init(argc, argv, "discrete_controller");
  ros::NodeHandle nh;

  //Gain controller
  nh.param(name_node + "/" + gain_string + "/" + "K1", k1, 1.0);
  nh.param(name_node + "/" + gain_string + "/" + "K2", k2, 1.0);
  nh.param(name_node + "/" + gain_string + "/" + "K3", k3, 1.0);
  
  ROS_INFO("Gain [k1: %f, k3: %f, k2: %f]", k1, k2, k3);
  path = new PathPlotter(nh, name_node, 2, 1000);
  path->setTime(10);
  path->setActionRate(rate_fnc);
  path->setActionMultiRate(multirate_fnc);
  path->setPathController(path_controller);

  ros::Subscriber odometry = nh.subscribe("/robot/odometry", 1000, odometry_Callback);
  ros::Subscriber goal = nh.subscribe("/move_base_simple/goal", 1000, poseCallback);

  ROS_INFO("Wait 2sec...");
  sleep(2);
  ROS_INFO("Ready!");
  //Start controller
  ROS_INFO("STARTED");
  ros::spin();


  return 0;
}

