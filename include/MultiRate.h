/* 
 * File:   MultiRate.h
 * Author: raffaello
 *
 * Created on July 25, 2013, 6:30 PM
 */

#ifndef MULTIRATE_H
#define	MULTIRATE_H

#include "ros/ros.h"
#include "std_msgs/String.h"

class MultiRate {
public:
    MultiRate(const ros::NodeHandle& nh, unsigned int time, unsigned int multirate);
    MultiRate(const MultiRate& orig);
    virtual ~MultiRate();
private:
    ros::Timer timer_;
    unsigned int time_, multirate_, counter_;
    ros::NodeHandle nh_;
    void timerCallback(const ros::TimerEvent& event);
};

#endif	/* MULTIRATE_H */

