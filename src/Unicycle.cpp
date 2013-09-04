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

void Unicycle::update()
{
  pose.theta += velocity.ang_vel;
  pose.x += cos(pose.theta) * velocity.lin_vel;
  pose.y += sin(pose.theta) * velocity.lin_vel;
}

void Unicycle::setVelocity(motion_control::Velocity vel)
{
  velocity = vel;
}

motion_control::Pose Unicycle::getPose()
{
  return pose;
}