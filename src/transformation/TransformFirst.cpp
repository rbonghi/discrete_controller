/* 
 * File:   Transform.cpp
 * Author: raffaello
 * 
 * Created on July 24, 2013, 7:30 PM
 */

#include "transformation/TransformFirst.h"
#include <math.h>
#include <serial_bridge/Pose.h>
#include <discrete_controller/Transform.h>
#include <discrete_controller/Command.h>

TransformFirst::TransformFirst() {
    state.z1 = 0;
    state.z2 = 0;
    state.z3 = 0;
}

TransformFirst::TransformFirst(const serial_bridge::Pose *pose) {
    state = transformPose(pose);
}

TransformFirst::TransformFirst(const geometry_msgs::PoseStamped *pose) {
    state = transformPoseStamped(pose);
}

TransformFirst::TransformFirst(const TransformFirst& orig) {
}

TransformFirst::~TransformFirst() {
}

discrete_controller::Transform TransformFirst::transformPose(const serial_bridge::Pose *pose) {
    discrete_controller::Transform state;
    state.z1 = pose->x;
    state.z2 = tan(pose->theta);
    state.z3 = pose->y;
    return state;
}

discrete_controller::Transform TransformFirst::transformPoseStamped(const geometry_msgs::PoseStamped *pose) {
    discrete_controller::Transform state;
    state.z1 = pose->pose.position.x;
    state.z2 = tf::getYaw(pose->pose.orientation);
    state.z3 = pose->pose.position.y;
    return state;
};

void TransformFirst::setPose(const serial_bridge::Pose *pose) {
    state = transformPose(pose);
}

void TransformFirst::setPoseStamped(const geometry_msgs::PoseStamped *pose) {
    state = transformPoseStamped(pose);
}

serial_bridge::Pose TransformFirst::antiTransform() {
    serial_bridge::Pose pose;
    pose.x = state.z1;
    pose.y = state.z3;
    pose.theta = atan(state.z2);
    return pose;
}

serial_bridge::Velocity TransformFirst::control(discrete_controller::Command cmd) {
    float costh = cos(antiTransform().theta);
    serial_bridge::Velocity velocity;
    velocity.lin_vel = cmd.u1 / costh;
    velocity.ang_vel = cmd.u2 * (pow(costh, 2));
    return velocity;
}

TransformFirst TransformFirst::operator-(const TransformFirst& p) {
    TransformFirst diff;
    diff.state.z1 = this->state.z1 - p.state.z1;
    diff.state.z2 = this->state.z2 - p.state.z2;
    diff.state.z3 = this->state.z3 - p.state.z3;
    return diff;
}

TransformFirst TransformFirst::operator/(const double& p) {
    TransformFirst div;
    div.state.z1 = this->state.z1 / p;
    div.state.z2 = this->state.z2 / p;
    div.state.z3 = this->state.z3 / p;
    return div;
}