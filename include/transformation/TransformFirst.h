/* 
 * File:   Transform.h
 * Author: raffaello
 *
 * Created on July 24, 2013, 7:30 PM
 */

#ifndef TRANSFORMFIRST_H
#define	TRANSFORMFIRST_H

#include <serial_bridge/Pose.h>
#include <serial_bridge/Velocity.h>
#include <discrete_controller/Transform.h>
#include <discrete_controller/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include "AbstractTransform.h"

class TransformFirst : public AbstractTransform {
public:
    TransformFirst();
    TransformFirst(const serial_bridge::Pose *pose);
    TransformFirst(const geometry_msgs::PoseStamped *pose);
    TransformFirst(const TransformFirst& orig);
    virtual ~TransformFirst();
    
    void setPose(const serial_bridge::Pose *pose);
    void setPoseStamped(const geometry_msgs::PoseStamped *pose);
    serial_bridge::Pose antiTransform();
    serial_bridge::Velocity control(discrete_controller::Command cmd);
    TransformFirst operator-(const TransformFirst& p);
    TransformFirst operator/(const double& p);
    
    discrete_controller::Transform state;
private:
    discrete_controller::Transform transformPose(const serial_bridge::Pose *pose);
    discrete_controller::Transform transformPoseStamped(const geometry_msgs::PoseStamped *pose);
};

#endif	/* TRANSFORM_H */

