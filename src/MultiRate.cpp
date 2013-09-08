/* 
 * File:   MultiRate.cpp
 * Author: raffaello
 * 
 * Created on July 25, 2013, 6:30 PM
 */

#include "MultiRate.h"

MultiRate::MultiRate(const ros::NodeHandle& nh, unsigned int time, unsigned int multirate) : nh_(nh)
{
  step_time_ = ((float) time) / multirate;
  ROS_INFO("step: %f", step_time_);
  time_ = time;
  multirate_ = multirate;
  counter_ = multirate;
  actionRate_ = NULL;
  actionStop_ = NULL;
  pActions = (ActionType *) malloc((multirate - 1) * sizeof (ActionType));
  for (int i = 0; i < multirate; i++)
    pActions[i] = NULL;
  counter_actions_ = 0;
  stop_ = false;

  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/discrete/goal", 1000);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/discrete/path", 1000);
}

MultiRate::MultiRate(const MultiRate& orig)
{
}

MultiRate::~MultiRate()
{
  free(pActions);
  pActions = NULL;
}

unsigned int MultiRate::getTime()
{
  return time_;
}

unsigned int MultiRate::getMultiRate()
{
  return multirate_;
}

void MultiRate::setActionRate(ActionType action)
{
  actionRate_ = action;
}

void MultiRate::setActionStop(ActionType action)
{
  actionStop_ = action;
}

void MultiRate::setActionMultiRate(ActionType action)
{
  //  ROS_INFO("Counter_actions: %d", counter_actions_);
  if (counter_actions_ <= (multirate_ - 2))
    pActions[counter_actions_++] = action;
  else
    ROS_ERROR("FULL");
}

void MultiRate::multiRateStep(int step)
{
  //  ROS_INFO("Multi Rate step, number: %d", step - 1);
  if (pActions[step - 1] == NULL)
  {
    ROS_ERROR("Never function on step: %d", step);
  }
  else
    pActions[step - 1]();
}

void MultiRate::rateStep()
{
  //  ROS_INFO("Rate step");
  if (actionRate_ == NULL)
    ROS_ERROR("Never rate function");
  else
    actionRate_();
}

void MultiRate::start()
{
  timer_ = nh_.createTimer(ros::Duration(step_time_), &MultiRate::timerCallback, this);
  rateStep();
  counter_ = 1;
}

void MultiRate::stop()
{
  if (actionStop_ == NULL)
    ROS_ERROR("Never stop function");
  else
    actionStop_();
}

void MultiRate::timerCallback(const ros::TimerEvent& event)
{
  ROS_INFO("counter: %d", counter_);
  if (!(counter_ % multirate_))
  {
    //        ROS_INFO("stop: %d", stop_);
    if (stop_)
    {
      timer_.stop();
      stop();
    }
    else
    {
      rateStep();
      counter_ = 0;
    }
  }
  else
  {
    multiRateStep(counter_);
    stop_ = true;
  }
  counter_++;
}

void MultiRate::plot_rate()
{
  motion_control::Pose ideal_pose;
  double clk = ((double) time_) / ((double) tot_step_);
  for (int i = start_rate_; i < finish_rate_; i++)
  {
    ideal_pose = ideal_trajectory_->getPose();
    Transform tran(&ideal_pose);
    motion_control::Velocity velocity = tran.control(*cmd_);
    ideal_trajectory_->setVelocity(velocity);
    ideal_trajectory_->update(clk);
    ideal_pose = ideal_trajectory_->getPose();
    ideal_path_.poses[i].pose.position.x = ideal_pose.x;
    ideal_path_.poses[i].pose.position.y = ideal_pose.y;
    ideal_path_.poses[i].pose.position.z = 0;
    //        ROS_INFO("x: %f, y: %f", ideal_pose.x, ideal_pose.y);
  }
  start_rate_ = finish_rate_;
  finish_rate_ += tot_step_ / multirate_;
}

void MultiRate::plot_path(int tot_step, motion_control::Pose pose_robot, discrete_controller::Command* cmd, const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tot_step_ = tot_step;
  cmd_ = cmd;
  start_rate_ = 0;
  finish_rate_ = tot_step_ / multirate_;
  ideal_path_.header.frame_id = msg.get()->header.frame_id;
  ideal_path_.poses.resize(tot_step_);
  ideal_trajectory_ = new Unicycle(pose_robot);

  for (int counter = 0; counter < multirate_; counter++)
  {
    //    ROS_INFO("Counter: %d", counter);
    if (!(counter % multirate_))
    {
      //      ROS_INFO("RATE STEP");
      rateStep();
    }
    else
    {
      //      ROS_INFO("MULTIRATE STEP");
      multiRateStep(counter);
    }
    plot_rate();
  }
  //  ROS_INFO("Exit");
  pub_path_.publish(ideal_path_);
  pub_goal_.publish(msg);
}