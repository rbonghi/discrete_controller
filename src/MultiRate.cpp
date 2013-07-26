/* 
 * File:   MultiRate.cpp
 * Author: raffaello
 * 
 * Created on July 25, 2013, 6:30 PM
 */

#include "MultiRate.h"

MultiRate::MultiRate(const ros::NodeHandle& nh, unsigned int time, unsigned int multirate) : nh_(nh)
{
  float step_time = ((float) time)/multirate;
  ROS_INFO("step: %f", step_time);
  timer_ = nh_.createTimer(ros::Duration(step_time), &MultiRate::timerCallback, this);
  time_ = time;
  multirate_ = multirate;
  counter_ = 0;
}

MultiRate::MultiRate(const MultiRate& orig)
{
}

MultiRate::~MultiRate()
{
}

void MultiRate::timerCallback(const ros::TimerEvent& event)
{
  ROS_INFO("I'M CALLBACK, counter: %d", counter_);
  if(counter_ % multirate_) {
    ROS_INFO("STEP RATE");
    timer_.stop();
  }
  counter_++;
}
