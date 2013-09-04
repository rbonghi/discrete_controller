/* 
 * File:   MultiRate.cpp
 * Author: raffaello
 * 
 * Created on July 25, 2013, 6:30 PM
 */

#include "MultiRate.h"

MultiRate::MultiRate(const ros::NodeHandle& nh, unsigned int time, unsigned int multirate) : nh_(nh) {
    float step_time = ((float) time) / multirate;
    ROS_INFO("step: %f", step_time);
    timer_ = nh_.createTimer(ros::Duration(step_time), &MultiRate::timerCallback, this);
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
}

MultiRate::MultiRate(const MultiRate& orig) {
}

MultiRate::~MultiRate() {
    free(pActions);
    pActions = NULL;
}

void MultiRate::setActionRate(ActionType action) {
    actionRate_ = action;
}

void MultiRate::setActionStop(ActionType action) {
    actionStop_ = action;
}

void MultiRate::setActionMultiRate(ActionType action) {
    //  ROS_INFO("Counter_actions: %d", counter_actions_);
    if (counter_actions_ <= (multirate_ - 2))
        pActions[counter_actions_++] = action;
    else
        ROS_ERROR("FULL");
}

void MultiRate::multiRateStep(int step) {
    //  ROS_INFO("Multi Rate step, number: %d", step - 1);
    if (pActions[step - 1] == NULL) {
        ROS_ERROR("Never function on step: %d", step);
    } else
        pActions[step - 1]();
}

void MultiRate::rateStep() {
    //  ROS_INFO("Rate step");
    if (actionRate_ == NULL)
        ROS_ERROR("Never rate function");
    else
        actionRate_();
}

void MultiRate::stop() {
    if (actionStop_ == NULL)
        ROS_ERROR("Never stop function");
    else
        actionStop_();
}

void MultiRate::timerCallback(const ros::TimerEvent& event) {
//    ROS_INFO("counter: %d", counter_);
    if (!(counter_ % multirate_)) {
//        ROS_INFO("stop: %d", stop_);
        if (stop_) {
            timer_.stop();
            stop();
        } else {
            rateStep();
            counter_ = 0;
        }
    } else {
        multiRateStep(counter_);
        stop_ = true;
    }
    counter_++;
}