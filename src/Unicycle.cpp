/* 
 * File:   Unicycle.cpp
 * Author: raffaello
 * 
 * Created on 02 September 2013, 15:35
 */

#include "Unicycle.h"

Unicycle::Unicycle()
{
  current_time_ = ros::Time::now();
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
  pose.pose.orientation = odom_quat;
  velocity.lin_vel = 0;
  velocity.ang_vel = 0;
}

Unicycle::Unicycle(geometry_msgs::PoseStamped pose)
{
  current_time_ = ros::Time::now();
  this->pose = pose;
  velocity.lin_vel = 0;
  velocity.ang_vel = 0;
}

Unicycle::Unicycle(const Unicycle& orig)
{
}

Unicycle::~Unicycle()
{
}

void Unicycle::update(ros::Duration duration)
{
  double rate_update = duration.toSec();
  //  ROS_INFO("rate: %f", rate_update);
  double th_old = tf::getYaw(pose.pose.orientation);
  double th_new = th_old + velocity.ang_vel*rate_update;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_new);
  current_time_ += duration;
  pose.header.stamp = current_time_;
  pose.pose.position.z = 0.0;
  if (velocity.ang_vel != 0)
  {
    pose.pose.position.x += (velocity.lin_vel) / (velocity.ang_vel) * (sin(th_new) - sin(th_old));
    pose.pose.position.y -= (velocity.lin_vel) / (velocity.ang_vel) * (cos(th_new) - cos(th_old));
  }
  else
  {
    pose.pose.position.x += velocity.lin_vel * rate_update * cos(th_old);
    pose.pose.position.y += velocity.lin_vel * rate_update * sin(th_old);
  }
}

void Unicycle::setVelocity(serial_bridge::Velocity vel)
{
  velocity = vel;
}

void Unicycle::setPose(geometry_msgs::PoseStamped pose)
{
  current_time_ = ros::Time::now();
  this->pose = pose;
}

geometry_msgs::PoseStamped Unicycle::getPose()
{
  return pose;
}

ros::Time Unicycle::getTime()
{
  return current_time_;
}