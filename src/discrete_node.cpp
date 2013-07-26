/* 
 * File:   discrete_node.cpp
 * Author: raffaello
 *
 * Created on July 24, 2013, 5:04 PM
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "MultiRate.h"
#include "Transform.h"

#include <cstdlib>
#include <nav_msgs/Odometry.h>
#include <motion_control/Velocity.h>
#include <motion_control/Pose.h>
#include <discrete_controller/Command.h>
#include <math.h>

using namespace std;
ros::Publisher pub_control, pub_pose;
motion_control::Pose pose_start, pose_stop;
discrete_controller::Command cmd;

void multirate_fnc()
{
  ROS_INFO("MultiRate Action");
}

void rate_fnc()
{
  ROS_INFO("Rate Action");
  discrete_controller::Transform start = Transform::TransformPose(pose_start);
  discrete_controller::Transform stop = Transform::TransformPose(pose_stop);
}

void chainedCallback(const motion_control::Pose::ConstPtr& msg)
{
  ROS_INFO("x: %f, y: %f, th: %f", msg.get()->x, msg.get()->y, msg.get()->theta);
  motion_control::Velocity velocity = Transform::control(cmd, msg.get());
  ROS_INFO("v: %f, a: %f", velocity.lin_vel, velocity.ang_vel);
  pub_control.publish(velocity);
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

  MultiRate* rate = new MultiRate(nh, 10, 2);
  pub_control = nh.advertise<motion_control::Velocity>("/robot/cmd_velocity", 1000);
  pub_pose = nh.advertise<motion_control::Pose>("/robot/cmd_pose", 1000);
  pose_start.x = -1;
  pose_start.y = 1;
  pose_start.theta = M_PI / 4;
  ROS_INFO("start [x: %f, y: %f, th: %f]", pose_start.x, pose_start.y, pose_start.theta);
  pose_stop.x = 0;
  pose_stop.y = 0;
  pose_stop.theta = M_PI / 4;
  ROS_INFO("stop [x: %f, y: %f, th: %f]", pose_stop.x, pose_stop.y, pose_stop.theta);

  sleep(2);
  ROS_INFO("!");
  ros::spinOnce();
  pub_pose.publish(pose_start);

  ros::Subscriber sub = nh.subscribe("/robot/pose", 1000, chainedCallback);

  rate->setActionRate(rate_fnc);
  rate->setActionMultiRate(multirate_fnc);

  ROS_INFO("Start controller");
  ros::spin();
  ROS_INFO("EXIT");

  return 0;
}

