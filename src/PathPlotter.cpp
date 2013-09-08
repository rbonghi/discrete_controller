/* 
 * File:   PathPlotter.cpp
 * Author: raffaello
 * 
 * Created on 08 September 2013, 17:04
 */

#include "PathPlotter.h"
#include "transformation/AbstractTransform.h"

PathPlotter::PathPlotter(const ros::NodeHandle& nh, std::string name, int multirate, int length) : nh_(nh) {
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + name + "/" + goal_string, 1000);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/" + name + "/" + path_string, 1000);
    multirate_ = multirate;
    length_ = length;
    path_.poses.resize(length);
    unicycle_ = new Unicycle();

    //counter_ = multirate;
    actionRate_ = NULL;
    actionStop_ = NULL;
    pActions = (ActionType *) malloc((multirate - 1) * sizeof (ActionType));
    for (int i = 0; i < multirate; i++)
        pActions[i] = NULL;
    counter_actions_ = 0;
}

PathPlotter::PathPlotter(const PathPlotter& orig) {
}

PathPlotter::~PathPlotter() {
    pub_goal_.~Publisher();
    pub_path_.~Publisher();
    unicycle_->~Unicycle();
}

void PathPlotter::setTime(int time) {
    this->time_ = time;
}

unsigned int PathPlotter::getTime() {
    return time_;
}

unsigned int PathPlotter::getMultiRate() {
    return multirate_;
}

void PathPlotter::setActionRate(ActionType action) {
    actionRate_ = action;
}

void PathPlotter::setActionStop(ActionType action) {
    actionStop_ = action;
}

void PathPlotter::setActionMultiRate(ActionType action) {
    //  ROS_INFO("Counter_actions: %d", counter_actions_);
    if (counter_actions_ <= (multirate_ - 2))
        pActions[counter_actions_++] = action;
    else
        ROS_ERROR("FULL");
}

discrete_controller::Command PathPlotter::rateStep(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal) {
    //  ROS_INFO("Rate step");
    if (actionRate_ == NULL) {
        ROS_ERROR("Never rate function");
        discrete_controller::Command cmd;
        cmd.u1 = 0;
        cmd.u2 = 0;
        return cmd;
    } else
        return actionRate_(time_, pose_robot, pose_goal);
}

discrete_controller::Command PathPlotter::multiRateStep(int step, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal) {
    //  ROS_INFO("Multi Rate step, number: %d", step - 1);
    if (pActions[step - 1] == NULL) {
        ROS_ERROR("Never function on step: %d", step);
        discrete_controller::Command cmd;
        cmd.u1 = 0;
        cmd.u2 = 0;
        return cmd;
    } else
        return pActions[step - 1](time_, pose_robot, pose_goal);
}

void PathPlotter::setGoal(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal, AbstractTransform* transform) {
    cmd_.u1 = 0;
    cmd_.u2 = 0;

    geometry_msgs::PoseStamped pose;
    pose.pose = pose_goal->pose;
    pose.header = pose_goal->header;
    pose = (geometry_msgs::PoseStamped&) pose_goal;

    ROS_INFO("Pose x:%f", pose_robot->pose.position.x);
    unicycle_->setPose((geometry_msgs::PoseStamped&)pose_robot);
    double step = length_ / ((double) multirate_);

    for (int counter = 0; counter < multirate_; counter++) {
        if (!(counter % multirate_)) {
            cmd_ = rateStep(pose_robot, pose_goal);
        } else {
            cmd_ = multiRateStep(counter, pose_robot, pose_goal);
        }
        draw_path(transform, counter, step);
    }
    pub_path_.publish(path_);
    pub_goal_.publish((geometry_msgs::PoseStamped&)pose_goal);
}

void PathPlotter::draw_path(AbstractTransform* transform, int counter, int step) {
    for (int i = counter * step; i < (counter + 1) * step; i++) {
        geometry_msgs::PoseStamped pose = unicycle_->getPose();
        transform->setPoseStamped(&pose);
        motion_control::Velocity velocity = transform->control(cmd_);
        unicycle_->setVelocity(velocity);
        unicycle_->update(step);
        path_.poses[i] = unicycle_->getPose();
    }
}
