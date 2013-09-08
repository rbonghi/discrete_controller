/* 
 * File:   discrete_controller.cpp
 * Author: raffaello
 *
 * Created on 08 September 2013, 16:54
 */

#include <cstdlib>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transformation/Transform.h"
#include "PathPlotter.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <motion_control/Velocity.h>
#include <motion_control/Pose.h>
#include <discrete_controller/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

PathPlotter* path;
Transform alpha;
discrete_controller::Command cmd;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("POSE");
//    AbstractTransform* transform = new Transform();
//    geometry_msgs::PoseStamped pose_robot;
//    pose_robot.pose.position.x = 0;
//    pose_robot.pose.position.y = 0;
//    path->setGoal(&pose_robot, msg.get(), transform);
    //  pose_k.x = msg.get()->pose.position.x;
    //  pose_k.y = msg.get()->pose.position.y;
    //  pose_k.theta = tf::getYaw(msg.get()->pose.orientation);
    //  ROS_INFO("Goal: [x: %f, y: %f, th: %f]", pose_k.x, pose_k.y, pose_k.theta);
}

discrete_controller::Command rate_fnc(int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal) {

    ROS_INFO("Rate Action");
    Transform zr(pose_robot);
    ROS_INFO("zr [z1: %f, z2: %f, z3: %f]", zr.state.z1, zr.state.z2, zr.state.z3);
    Transform zk(pose_goal);
    ROS_INFO("zk [z1: %f, z2: %f, z3: %f]", zk.state.z1, zk.state.z2, zk.state.z3);
    alpha = (zk - zr) / delta;
    ROS_INFO("alpha [z1: %f, z2: %f, z3: %f]", alpha.state.z1, alpha.state.z2, alpha.state.z3);

    cmd.u1 = alpha.state.z1;
    cmd.u2 = (-alpha.state.z2 + (4 * alpha.state.z3) / (delta * alpha.state.z1) - (4 * zr.state.z2) / (delta));

    ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
    return cmd;
}

discrete_controller::Command multirate_fnc(int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal) {
    ROS_INFO("MultiRate Action");
    double u21 = cmd.u2;
    cmd.u2 = (2 * alpha.state.z2 - u21);

    ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
    return cmd;
}

/*
 * 
 */
int main(int argc, char** argv) {

    //Init the serial_motion_node
    ros::init(argc, argv, "discrete_controller");
    ros::NodeHandle nh;

    path = new PathPlotter(nh, "discrete", 2, 1000);
    path->setTime(10);
    path->setActionRate(rate_fnc);
    path->setActionMultiRate(multirate_fnc);

    ros::Subscriber goal = nh.subscribe("/move_base_simple/goal", 1000, poseCallback);

    //Start controller
    ROS_INFO("STARTED");
    ros::spin();


    return 0;
}

