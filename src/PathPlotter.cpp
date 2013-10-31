/* 
 * File:   PathPlotter.cpp
 * Author: raffaello
 * 
 * Created on 08 September 2013, 17:04
 */

#include "PathPlotter.h"
#include "transformation/AbstractTransform.h"

PathPlotter::PathPlotter(const ros::NodeHandle& nh, std::string name, int multirate, int length) : nh_(nh)
{
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + name + "/" + goal_string, 1000);
  pub_array_step_ = nh_.advertise<geometry_msgs::PoseArray>("/" + name + "/" + pose_array_string, 1000);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/" + name + "/" + path_string, 1000);
  pub_multirate_ = nh_.advertise<discrete_controller::Command>("/" + name + "/" + command_string, 1000);
  pub_desidered_unicycle_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + name + "/" + desidered_unicycle_string, 1000);
  //Emergency stop control
  control_stop_srv_ = nh_.advertiseService(control_stop_string, &PathPlotter::control_stop_Callback, this);
  name_ = name;
  multirate_ = multirate;
  length_ = length;
  path_.poses.resize(length);
  unicycle_ = new Unicycle();

  actionRate_ = NULL;
  actionStop_ = NULL;
  pActions = (ActionType *) malloc((multirate - 1) * sizeof (ActionType));
  for (int i = 0; i < multirate; i++)
    pActions[i] = NULL;
  counter_actions_ = 0;
  stop_ = false;

  //path controller
  controller_ == NULL;
  transform_controller_ = NULL;

  //parameter step trajectory
  length_step_ = 2;
  nh_.setParam(name + "/" + pose_array_string, length_step_);
}

PathPlotter::PathPlotter(const PathPlotter& orig)
{
}

PathPlotter::~PathPlotter()
{
  pub_goal_.~Publisher();
  pub_path_.~Publisher();
  unicycle_->~Unicycle();
  free(pActions);
  pActions = NULL;
}

void PathPlotter::setTime(int time)
{
  this->time_ = time;
}

unsigned int PathPlotter::getTime()
{
  return time_;
}

unsigned int PathPlotter::getMultiRate()
{
  return multirate_;
}

void PathPlotter::setPathController(ControllerType controller)
{
  controller_ = controller;
}

void PathPlotter::setActionRate(ActionType action)
{
  actionRate_ = action;
}

void PathPlotter::setActionStop(ActionType action)
{
  actionStop_ = action;
}

void PathPlotter::setActionMultiRate(ActionType action)
{
  //  ROS_INFO("Counter_actions: %d", counter_actions_);
  if (counter_actions_ <= (multirate_ - 2))
    pActions[counter_actions_++] = action;
  else
    ROS_ERROR("FULL");
}

discrete_controller::Command PathPlotter::stop(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal)
{
  sub_odometry_.shutdown();
  if (actionStop_ == NULL)
  {
    ROS_ERROR("Never stop function");
    discrete_controller::Command cmd;
    cmd.u1 = 0;
    cmd.u2 = 0;
    return cmd;
  }
  else
    return actionStop_(time_, pose_robot, pose_goal);
}

discrete_controller::Command PathPlotter::rateStep(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal)
{
  //  ROS_INFO("Rate step");
  if (actionRate_ == NULL)
  {
    ROS_ERROR("Never rate function");
    discrete_controller::Command cmd;
    cmd.u1 = 0;
    cmd.u2 = 0;
    return cmd;
  }
  else
  {
    return actionRate_(time_, pose_robot, pose_goal);
  }
}

discrete_controller::Command PathPlotter::multiRateStep(int step, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal)
{
  //  ROS_INFO("Multi Rate step, number: %d", step - 1);
  if (pActions[step - 1] == NULL)
  {
    ROS_ERROR("Never function on step: %d", step);
    discrete_controller::Command cmd;
    cmd.u1 = 0;
    cmd.u2 = 0;
    return cmd;
  }
  else
  {
    return pActions[step - 1](time_, pose_robot, pose_goal);
  }
}

void PathPlotter::timerCallback(const ros::TimerEvent& event)
{
  //  ROS_INFO("counter: %d", counter_);
  if (!(counter_ % multirate_))
  {
    //        ROS_INFO("stop: %d", stop_);
    if (stop_)
    {
      timer_.stop();
      cmd_controller_ = stop(pose_robot_controller_, pose_goal_controller_);
      serial_bridge::Velocity velocity = transform_controller_->control(cmd_controller_);
      pub_path_control_.publish(velocity);
    }
    else
    {
      ROS_INFO("Rate Action");
      cmd_controller_ = rateStep(pose_robot_controller_, pose_goal_controller_);
      ROS_INFO("Step: %d [u1: %f, u2: %f]", counter_, cmd_controller_.u1, cmd_controller_.u2);
      counter_ = 0;

    }
  }
  else
  {
    ROS_INFO("Multirate action N: %d", counter_);
    cmd_controller_ = multiRateStep(counter_, pose_robot_controller_, pose_goal_controller_);
    ROS_INFO("Step: %d [u1: %f, u2: %f]", counter_, cmd_controller_.u1, cmd_controller_.u2);
    stop_ = true;
  }
  counter_++;
}

void PathPlotter::odometry_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ros::Duration update = msg.get()->header.stamp - unicycle_->getTime();
  //    ROS_INFO("Odometry diff [sec: %d, nsec: %d]", update.sec, update.nsec);

  pub_multirate_.publish(cmd_controller_); //Publishing chained command control
  //Generator desidered position
  geometry_msgs::PoseStamped pose_temp = unicycle_->getPose();
  transform_controller_->setPoseStamped(&pose_temp);
  serial_bridge::Velocity velocityd = transform_controller_->control(cmd_controller_);
  unicycle_->setVelocity(velocityd);
  unicycle_->update(update);
  geometry_msgs::PoseStamped posed = unicycle_->getPose();
  pub_desidered_unicycle_.publish(posed);
  // Start control path
  serial_bridge::Velocity velocity = path_controller(velocityd, posed, *msg.get());
  pub_path_control_.publish(velocity);
}

serial_bridge::Velocity PathPlotter::path_controller(serial_bridge::Velocity velocityd, geometry_msgs::PoseStamped posed, nav_msgs::Odometry pose_robot)
{
  if (controller_ == NULL)
  {
    ROS_ERROR("Never path controller function");
    serial_bridge::Velocity velocity;
    velocity.lin_vel = 0;
    velocity.ang_vel = 0;
    return velocity;
  }
  else
  {
    return controller_(nh_, velocityd, posed, pose_robot);
  }
}

void PathPlotter::startController(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal, AbstractTransform* transform, std::string robot, std::string odometry, std::string velocity)
{
  if (stop_)
  {
    timer_.stop();
    sub_odometry_.shutdown();
  }
  counter_ = 0;
  pose_robot_controller_ = pose_robot;
  pose_goal_controller_ = pose_goal;
  transform_controller_ = transform;
  sub_odometry_ = nh_.subscribe("/" + robot + "/" + odometry, 1000, &PathPlotter::odometry_Callback, this);
  pub_path_control_ = nh_.advertise<serial_bridge::Velocity>("/" + robot + "/" + velocity, 1000);
  geometry_msgs::PoseStamped pose;
  pose.pose = pose_robot->pose;
  pose.header = pose_robot->header;
  path_.header = pose.header;
  unicycle_->setPose(pose);

  //Start Controller
  stop_ = false;
  double rate_time = time_ / ((double) multirate_);
  timer_ = nh_.createTimer(ros::Duration(rate_time), &PathPlotter::timerCallback, this);
  cmd_controller_ = rateStep(pose_robot_controller_, pose_goal_controller_);
  ROS_INFO("Step: %d [u1: %f, u2: %f]", counter_, cmd_controller_.u1, cmd_controller_.u2);
  counter_ = 1;

}

void PathPlotter::setGoal(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal, AbstractTransform* transform)
{
  discrete_controller::Command cmd; //Command to drive robot
  geometry_msgs::PoseStamped pose;
  pose.pose = pose_robot->pose;
  pose.header = pose_robot->header;
  path_.header = pose.header;
  unicycle_->setPose(pose);
  double step = length_ / multirate_;
  double update = time_ / ((double) length_);
  //Print step
  counter_step_pose_array_ = 0;
  array_step_.header = pose_robot->header;
  nh_.getParam(name_ + "/" + pose_array_string, length_step_); //Update step
  array_step_.poses.resize(length_step_);
  step_pose_array_ = length_ / length_step_;
  for (int counter = 0; counter < multirate_; counter++)
  {
    if (!(counter % multirate_))
    {
      cmd = rateStep(pose_robot, pose_goal);
    }
    else
    {
      cmd = multiRateStep(counter, pose_robot, pose_goal);
    }
    draw_path(transform, cmd, counter, step, update);
  }
  pub_path_.publish(path_);
  pub_goal_.publish(*pose_goal);
  ROS_INFO("Size: %d, Length: %d", length_step_, counter_step_pose_array_);
  pub_array_step_.publish(array_step_);
}

void PathPlotter::draw_path(AbstractTransform* transform, discrete_controller::Command cmd, int counter, int step, double update)
{
  for (int i = counter * step; i <= (counter + 1) * step; i++)
  {
    geometry_msgs::PoseStamped pose = unicycle_->getPose();
    transform->setPoseStamped(&pose);
    serial_bridge::Velocity velocity = transform->control(cmd);
    unicycle_->setVelocity(velocity);
    unicycle_->update(ros::Duration(update));
    if (i < (counter + 1) * step)
    {
      path_.poses[i] = unicycle_->getPose();
    }
    // Add pose array step
    if (i == (counter_step_pose_array_ + 1) * step_pose_array_)
    {
      ROS_INFO("Pose [%f, %f]", unicycle_->getPose().pose.position.x, unicycle_->getPose().pose.position.y);
      array_step_.poses[counter_step_pose_array_] = unicycle_->getPose().pose;
      counter_step_pose_array_++;
    }
  }
}

bool PathPlotter::control_stop_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("EMERGENCY STOP!");
  stop_ = false;
  timer_.stop();
  cmd_controller_ = stop(pose_robot_controller_, pose_goal_controller_);
  if (transform_controller_ != NULL)
  {
    serial_bridge::Velocity velocity = transform_controller_->control(cmd_controller_);
    pub_path_control_.publish(velocity);
  }
  return true;
}