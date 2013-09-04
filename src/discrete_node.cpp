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
ros::Publisher pub_goal, pub_path;
ros::Publisher pub_control, pub_pose, pub_multirate;
motion_control::Pose pose_robot, pose_k;
discrete_controller::Command cmd;
double delta;
Transform alpha;

void multirate_fnc() {
    ROS_INFO("MultiRate Action");
    double u21 = cmd.u2;
    cmd.u2 = (2 * alpha.state.z2 - u21);

    ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
}

void init_fnc() {
    ROS_INFO("Stop");
    cmd.u1 = 0;
    cmd.u2 = 0;
    motion_control::Velocity velocity;
    velocity.ang_vel = 0;
    velocity.ang_vel = 0;
    pub_control.publish(velocity);
    pub_multirate.publish(cmd);
    ros::shutdown();
}

void rate_fnc() {
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

void chainedCallback(const motion_control::Pose::ConstPtr& msg) {
    //    ROS_INFO("x: %f, y: %f, th: %f", msg.get()->x, msg.get()->y, msg.get()->theta);
    Transform tran(msg.get());
    motion_control::Velocity velocity = tran.control(cmd);
//    ROS_INFO("v: %f, a: %f", velocity.lin_vel, velocity.ang_vel);
    pub_control.publish(velocity);
    pub_multirate.publish(cmd);
}

void chained_odometry_Callback(const nav_msgs::Odometry::ConstPtr& msg) {
    double x = msg.get()->pose.pose.position.x;
    double y = msg.get()->pose.pose.position.y;
    double theta = tf::getYaw(msg.get()->pose.pose.orientation);
    ROS_INFO("Now - [x: %f, y: %f, th: %f]", x, y, theta);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  motion_control::Pose pose;
  pose.x = msg.get()->pose.position.x;
  pose.y = msg.get()->pose.position.y;
  pose.theta = tf::getYaw(msg.get()->pose.orientation);
  ROS_INFO("Goal: [x: %f, y: %f, th: %f]", pose.x, pose.y, pose.theta);
  pub_goal.publish(msg);


  nav_msgs::Path path;

  path.header.frame_id = "base_link";
  path.poses.resize(10);
  for (int i = 0; i < 10; i++)
  {
    path.poses[i].pose.position.x = 0.1 * i;
    path.poses[i].pose.position.y = 0.1 * i;
    path.poses[i].pose.position.z = 0;
  }
  pub_path.publish(path);
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

  // Create multirate controller
    MultiRate* rate = new MultiRate(nh, 10, 2);
  // Open publisher
  // command velocity robot
  pub_control = nh.advertise<motion_control::Velocity>("/robot/cmd_velocity", 1000);
  // command pose robot
  pub_pose = nh.advertise<motion_control::Pose>("/robot/cmd_pose", 1000);
  // rviz get goal
  ros::Subscriber goal = nh.subscribe("/move_base_simple/goal", 1000, poseCallback);
  // print goal and path
  pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/discrete/goal", 1000);
  pub_path = nh.advertise<nav_msgs::Path>("/discrete/path", 1000);

    sleep(2);
    ROS_INFO("!");
    ros::spinOnce();
    pub_pose.publish(pose_robot);

    ros::Subscriber sub = nh.subscribe("/robot/pose", 1000, chainedCallback);
    //  ros::Subscriber sub_odometry = nh.subscribe("/robot/odometry", 1000, chained_odometry_Callback);

    rate->setActionRate(rate_fnc);
    rate->setActionMultiRate(multirate_fnc);
    rate->setActionMultiRate(init_fnc);
    rate->setActionStop(init_fnc);

    ROS_INFO("Start controller");
    ros::spin();
    ROS_INFO("EXIT");

  return 0;
}

