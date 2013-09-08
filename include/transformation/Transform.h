/* 
 * File:   Transform.h
 * Author: raffaello
 *
 * Created on 27 July 2013, 11:44
 */

#ifndef TRANSFORM_H
#define	TRANSFORM_H

#include <motion_control/Pose.h>
#include <motion_control/Velocity.h>
#include <discrete_controller/Transform.h>
#include <discrete_controller/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include "AbstractTransform.h"

class Transform : public AbstractTransform {
public:
    Transform();
    Transform(const motion_control::Pose *pose);
    Transform(const geometry_msgs::PoseStamped *pose);
    Transform(const Transform& orig);
    virtual ~Transform();
    void setPose(const motion_control::Pose *pose);
    void setPoseStamped(const geometry_msgs::PoseStamped *pose);
    motion_control::Velocity control(discrete_controller::Command cmd);
    Transform operator-(const Transform& p);
    Transform operator/(const double& p);

    discrete_controller::Transform state;
private:
    discrete_controller::Transform transformPose(const motion_control::Pose *pose);
    discrete_controller::Transform transformPoseStamped(const geometry_msgs::PoseStamped *pose);
};

#endif	/* TRANSFORM_H */

