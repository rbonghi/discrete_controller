/* 
 * File:   Transform.cpp
 * Author: raffaello
 * 
 * Created on July 24, 2013, 7:30 PM
 */

#include "transformation/TransformFirst.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <discrete_controller/Transform.h>
#include <discrete_controller/Command.h>

TransformFirst::TransformFirst() {
    state.z1 = 0;
    state.z2 = 0;
    state.z3 = 0;
}

TransformFirst::TransformFirst(const nav_msgs::Odometry *pose) {
    state = transformPose(pose);
}

TransformFirst::TransformFirst(const geometry_msgs::PoseStamped *pose) {
    state = transformPoseStamped(pose);
}

TransformFirst::TransformFirst(const TransformFirst& orig) {
}

TransformFirst::~TransformFirst() {
}

discrete_controller::Transform TransformFirst::transformPose(const nav_msgs::Odometry *pose) {
    discrete_controller::Transform state;
    double theta = tf::getYaw(pose->pose.pose.orientation);
    state.z1 = pose->pose.pose.position.x;
    state.z2 = tan(theta);
    state.z3 = pose->pose.pose.position.y;
    return state;
}

discrete_controller::Transform TransformFirst::transformPoseStamped(const geometry_msgs::PoseStamped *pose) {
    discrete_controller::Transform state;
    state.z1 = pose->pose.position.x;
    state.z2 = tf::getYaw(pose->pose.orientation);
    state.z3 = pose->pose.position.y;
    return state;
};

void TransformFirst::setPose(const nav_msgs::Odometry *pose) {
    state = transformPose(pose);
}

void TransformFirst::setPoseStamped(const geometry_msgs::PoseStamped *pose) {
    state = transformPoseStamped(pose);
}

nav_msgs::Odometry TransformFirst::antiTransform() {
    nav_msgs::Odometry pose;
    pose.pose.pose.position.x = state.z1;
    pose.pose.pose.position.y = state.z3;
    pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan(state.z2));
    return pose;
}

geometry_msgs::Twist TransformFirst::control(discrete_controller::Command cmd) {
    double theta = tf::getYaw(antiTransform().pose.pose.orientation);
    float costh = cos(theta);
    geometry_msgs::Twist velocity;
    velocity.linear.x = cmd.u1 / costh;
    velocity.angular.z = cmd.u2 * (pow(costh, 2));
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
