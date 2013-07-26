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

#include <vector>

class MultiRate {
public:
    typedef void( * ActionType) (void);
    MultiRate(const ros::NodeHandle& nh, unsigned int time, unsigned int multirate);
    MultiRate(const MultiRate& orig);
    virtual ~MultiRate();
    
    void setActionRate(ActionType action);
    void setActionMultiRate(ActionType action);
private:
    ros::Timer timer_;
    unsigned int time_, multirate_, counter_;
    ros::NodeHandle nh_;
    ActionType *pActions;
    ActionType actionRate_;
    unsigned int counter_actions_;
    
    void timerCallback(const ros::TimerEvent& event);
    
    void multiRateStep(int step);
    void rateStep();
};

#endif	/* MULTIRATE_H */

