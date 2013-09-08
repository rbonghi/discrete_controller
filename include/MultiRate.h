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
#include "Unicycle.h"
#include "transformation/Transform.h"

#include <nav_msgs/Path.h>
#include <discrete_controller/Command.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

class MultiRate {
public:
    typedef void( * ActionType) (void);
    MultiRate(const ros::NodeHandle& nh, unsigned int time, unsigned int multirate);
    MultiRate(const MultiRate& orig);
    virtual ~MultiRate();

    unsigned int getTime();
    unsigned int getMultiRate();
    void setActionStop(ActionType action);
    void setActionRate(ActionType action);
    void setActionMultiRate(ActionType action);
    void start();

    void plot_path(int tot_step, motion_control::Pose pose_robot, discrete_controller::Command* cmd, const geometry_msgs::PoseStamped::ConstPtr& msg);
private:
    float step_time_;
    bool stop_;
    ros::Timer timer_;
    unsigned int time_, multirate_, counter_;
    ros::NodeHandle nh_;
    ActionType *pActions;
    ActionType actionRate_, actionStop_;
    unsigned int counter_actions_;

    ros::Publisher pub_goal_, pub_path_;
    int start_rate_, finish_rate_;
    int tot_step_;
    nav_msgs::Path ideal_path_;
    discrete_controller::Command* cmd_;
    Unicycle* ideal_trajectory_;
    void plot_rate();

    void timerCallback(const ros::TimerEvent& event);

    void multiRateStep(int step);
    void rateStep();
    void stop();
};

#endif	/* MULTIRATE_H */
