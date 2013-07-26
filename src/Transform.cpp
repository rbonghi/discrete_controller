/* 
 * File:   Transform.cpp
 * Author: raffaello
 * 
 * Created on July 24, 2013, 7:30 PM
 */

#include "Transform.h"
#include <math.h>
#include <motion_control/Pose.h>
#include <discrete_controller/Transform.h>

Transform::Transform(motion_control::Pose pose)
{
  state = TransformPose(pose);
}

Transform::Transform(const Transform& orig)
{
}

Transform::~Transform()
{
}

motion_control::Pose Transform::antiTrasform()
{
  return antiTransformPose(this->state);
}

discrete_controller::Transform Transform::TransformPose(motion_control::Pose pose)
{
  discrete_controller::Transform state;
  state.z1 = pose.x;
  state.z2 = tan(pose.theta);
  state.z3 = pose.y;
  return state;
}

motion_control::Pose Transform::antiTransformPose(discrete_controller::Transform state)
{
  motion_control::Pose pose;
  pose.x = state.z1;
  pose.y = state.z3;
  pose.theta = atan(state.z2);
  return pose;
}