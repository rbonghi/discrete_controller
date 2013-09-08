/* 
 * File:   AbstractTransform.h
 * Author: raffaello
 *
 * Created on 27 July 2013, 15:43
 */

#ifndef ABSTRACTTRANSFORM_H
#define	ABSTRACTTRANSFORM_H

#include <motion_control/Pose.h>
#include <motion_control/Velocity.h>
#include <discrete_controller/Transform.h>
#include <discrete_controller/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

class AbstractTransform {
public:
    virtual motion_control::Velocity control(discrete_controller::Command cmd){};
    virtual void setPose(const motion_control::Pose *pose){};
    virtual void setPoseStamped(const geometry_msgs::PoseStamped *pose){};
private:
    virtual discrete_controller::Transform transformPose(const motion_control::Pose *pose){};
    virtual discrete_controller::Transform transformPoseStamped(const geometry_msgs::PoseStamped *pose){};
};

#endif	/* ABSTRACTTRANSFORM_H */

