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

//#include <serial_bridge/Pose.h>
//#include <serial_bridge/Velocity.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class Unicycle {
public:
    Unicycle();
    Unicycle(geometry_msgs::PoseStamped pose);
    Unicycle(const Unicycle& orig);
    virtual ~Unicycle();
    void update(ros::Duration duration);
    void setVelocity(geometry_msgs::Twist vel);
    void setPose(geometry_msgs::PoseStamped pose);
    geometry_msgs::PoseStamped getPose();
    ros::Time getTime();
private:
    geometry_msgs::Twist velocity;
    //serial_bridge::Pose pose;
    ros::Time current_time_;    //ROS time
    geometry_msgs::PoseStamped pose;
};

#endif	/* UNICYCLE_H */

