/* 
 * File:   Transform.cpp
 * Author: raffaello
 * 
 * Created on 27 July 2013, 11:44
 */

#include "transformation/Transform.h"

Transform::Transform() {
    state.z1 = 0;
    state.z2 = 0;
    state.z3 = 0;
}

Transform::Transform(const motion_control::Pose *pose) {
    state = transformPose(pose);
}

Transform::Transform(const geometry_msgs::PoseStamped *pose) {
    state = transformPoseStamped(pose);
}

Transform::Transform(const Transform& orig) {
}

Transform::~Transform() {
}

void Transform::setPose(const motion_control::Pose *pose) {
    state = transformPose(pose);
}

void Transform::setPoseStamped(const geometry_msgs::PoseStamped *pose) {
    state = transformPoseStamped(pose);
}

discrete_controller::Transform Transform::transformPose(const motion_control::Pose *pose) {
    discrete_controller::Transform state;
    double costh = cos(pose->theta);
    double sinth = sin(pose->theta);
    state.z1 = pose->theta;
    state.z2 = pose->x * costh + pose->y * sinth;
    state.z3 = pose->x * sinth - pose->y * costh;
    return state;
}

discrete_controller::Transform Transform::transformPoseStamped(const geometry_msgs::PoseStamped *pose) {
    discrete_controller::Transform state;
    double theta = tf::getYaw(pose->pose.orientation);
    double costh = cos(theta);
    double sinth = sin(theta);
    state.z1 = theta;
    state.z2 = pose->pose.position.x * costh + pose->pose.position.y * sinth;
    state.z3 = pose->pose.position.x * sinth - pose->pose.position.y * costh;
    return state;
};

motion_control::Velocity Transform::control(discrete_controller::Command cmd) {
    motion_control::Velocity velocity;
    velocity.lin_vel = cmd.u2 + state.z3 * cmd.u1;
    velocity.ang_vel = cmd.u1;
    return velocity;
}

Transform Transform::operator-(const Transform& p) {
    Transform diff;
    diff.state.z1 = this->state.z1 - p.state.z1;
    diff.state.z2 = this->state.z2 - p.state.z2;
    diff.state.z3 = this->state.z3 - p.state.z3;
    return diff;
}

Transform Transform::operator/(const double& p) {
    Transform div;
    div.state.z1 = this->state.z1 / p;
    div.state.z2 = this->state.z2 / p;
    div.state.z3 = this->state.z3 / p;
    return div;
}