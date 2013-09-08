/* 
 * File:   Unicycle.cpp
 * Author: raffaello
 * 
 * Created on 02 September 2013, 15:35
 */

#include "Unicycle.h"

Unicycle::Unicycle()
{
  pose.x = 0;
  pose.y = 0;
  pose.theta = 0;
  velocity.lin_vel = 0;
  velocity.ang_vel = 0;
}

Unicycle::Unicycle(motion_control::Pose pose)
{
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

void Unicycle::update(double rate_update)
{
  double th_old = pose.theta;
  pose.theta += velocity.ang_vel*rate_update;
  if (velocity.ang_vel != 0)
  {
    pose.x += (velocity.lin_vel) / (velocity.ang_vel) * (sin(pose.theta) - sin(th_old));
    pose.y -= (velocity.lin_vel) / (velocity.ang_vel) * (cos(pose.theta) - cos(th_old));
  }
  else
  {
    pose.x += velocity.lin_vel*rate_update*cos(th_old);
    pose.y += velocity.lin_vel*rate_update*sin(th_old);
  }
}

void Unicycle::setVelocity(motion_control::Velocity vel)
{
  velocity = vel;
}

motion_control::Pose Unicycle::getPose()
{
  return pose;
}