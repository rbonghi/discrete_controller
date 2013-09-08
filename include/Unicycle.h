/* 
 * File:   Unicycle.h
 * Author: raffaello
 *
 * Created on 02 September 2013, 15:35
 */

#ifndef UNICYCLE_H
#define	UNICYCLE_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <motion_control/Pose.h>
#include <motion_control/Velocity.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class Unicycle {
public:
    Unicycle();
    Unicycle(geometry_msgs::PoseStamped pose);
    Unicycle(const Unicycle& orig);
    virtual ~Unicycle();
    void update(double rate_update);
    void setVelocity(motion_control::Velocity vel);
    void setPose(geometry_msgs::PoseStamped pose);
    geometry_msgs::PoseStamped getPose();
private:
    motion_control::Velocity velocity;
    //motion_control::Pose pose;
    ros::Time current_time_;    //ROS time
    geometry_msgs::PoseStamped pose;
};

#endif	/* UNICYCLE_H */

