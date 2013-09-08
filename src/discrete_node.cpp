/* 
 * File:   discrete_node.cpp
 * Author: raffaello
 *
 * Created on July 24, 2013, 5:04 PM
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "MultiRate.h"
#include "transformation/Transform.h"
#include "Unicycle.h"

#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <motion_control/Velocity.h>
#include <motion_control/Pose.h>
#include <discrete_controller/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>

using namespace std;
ros::Publisher pub_control, pub_pose, pub_multirate;
motion_control::Pose pose_robot, pose_k;
discrete_controller::Command cmd;
double delta, number_rate, rate_update;
Transform alpha;
MultiRate* rate;
bool control = false;
Unicycle* real_unicycle;

void multirate_fnc()
{
  ROS_INFO("MultiRate Action");
  double u21 = cmd.u2;
  cmd.u2 = (2 * alpha.state.z2 - u21);

  ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
}

void init_fnc()
{
  ROS_INFO("Stop");
  cmd.u1 = 0;
  cmd.u2 = 0;
  motion_control::Velocity velocity;
  velocity.ang_vel = 0;
  velocity.ang_vel = 0;
  pub_control.publish(velocity);
  pub_multirate.publish(cmd);
  //  ros::shutdown();
  control = false;
}

void rate_fnc()
{
  ROS_INFO("Rate Action");
  Transform zr(&pose_robot);
  ROS_INFO("zr [z1: %f, z2: %f, z3: %f]", zr.state.z1, zr.state.z2, zr.state.z3);
  Transform zk(&pose_k);
  ROS_INFO("zk [z1: %f, z2: %f, z3: %f]", zk.state.z1, zk.state.z2, zk.state.z3);
  alpha = (zk - zr) / delta;
  ROS_INFO("alpha [z1: %f, z2: %f, z3: %f]", alpha.state.z1, alpha.state.z2, alpha.state.z3);

  cmd.u1 = alpha.state.z1;
  cmd.u2 = (-alpha.state.z2 + (4 * alpha.state.z3) / (delta * alpha.state.z1) - (4 * zr.state.z2) / (delta));

  ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
}

void chainedCallback(const motion_control::Pose::ConstPtr& msg)
{
  motion_control::Pose pose_robot;
  pose_robot.x = msg.get()->x;
  pose_robot.y = msg.get()->y;
  pose_robot.theta = msg.get()->theta;
  if (control)
  {
    //    ROS_INFO("x: %f, y: %f, th: %f", msg.get()->x, msg.get()->y, msg.get()->theta);
//    Transform tran(msg.get());
    pose_robot = real_unicycle->getPose();
    Transform tran(&pose_robot);
    motion_control::Velocity velocity = tran.control(cmd);
    real_unicycle->setVelocity(velocity);
    real_unicycle->update(1/rate_update);
    pose_robot = real_unicycle->getPose();
    //    ROS_INFO("v: %f, a: %f", velocity.lin_vel, velocity.ang_vel);
    //    pub_control.publish(velocity);
    pub_multirate.publish(cmd);
  }
}

void chained_odometry_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double x = msg.get()->pose.pose.position.x;
  double y = msg.get()->pose.pose.position.y;
  double theta = tf::getYaw(msg.get()->pose.pose.orientation);
  ROS_INFO("Now - [x: %f, y: %f, th: %f]", x, y, theta);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose_k.x = msg.get()->pose.position.x;
  pose_k.y = msg.get()->pose.position.y;
  pose_k.theta = tf::getYaw(msg.get()->pose.orientation);
  ROS_INFO("Goal: [x: %f, y: %f, th: %f]", pose_k.x, pose_k.y, pose_k.theta);
  // Plot path and goal
  rate->plot_path(1000, pose_robot, &cmd, msg);
  rate->start();
  control = true;
  ROS_INFO("Start!");
  //Test
  real_unicycle = new Unicycle(pose_robot);
}

/*
 * 
 */
int main(int argc, char** argv)
{
  //Init the serial_motion_node
  ros::init(argc, argv, "discrete_node");
  ros::NodeHandle nh;

  cmd.u1 = 0;
  cmd.u2 = 0;

  rate_update = 50;

  delta = 20;
  number_rate = 2;
  // Create multirate controller
  rate = new MultiRate(nh, delta, number_rate);
  rate->setActionRate(rate_fnc);
  rate->setActionMultiRate(multirate_fnc);
  rate->setActionStop(init_fnc);
  // Open publisher
  // command velocity robot
  pub_control = nh.advertise<motion_control::Velocity>("/robot/cmd_velocity", 1000);
  // command pose robot
  pub_pose = nh.advertise<motion_control::Pose>("/robot/cmd_pose", 1000);
  // command multirate
  pub_multirate = nh.advertise<discrete_controller::Command>("/control/command", 1000);
  // rviz get goal
  ros::Subscriber goal = nh.subscribe("/move_base_simple/goal", 1000, poseCallback);

  ROS_INFO("Wait 2sec...");
  sleep(2);
  ROS_INFO("Ready!");
  ros::spinOnce();
  pub_pose.publish(pose_robot);

  ros::Subscriber sub = nh.subscribe("/robot/pose", 1000, chainedCallback);
  //  ros::Subscriber sub_odometry = nh.subscribe("/robot/odometry", 1000, chained_odometry_Callback);

  ROS_INFO("Initialized controller");
  ros::spin();
  ROS_INFO("EXIT");

  return 0;
}

