/* 
 * File:   Transform.h
 * Author: raffaello
 *
 * Created on July 24, 2013, 7:30 PM
 */

#ifndef TRANSFORM_H
#define	TRANSFORM_H

#include <motion_control/Pose.h>
#include <motion_control/Velocity.h>
#include <discrete_controller/Transform.h>
#include <discrete_controller/Command.h>

class Transform {
public:
    Transform(motion_control::Pose pose);
    Transform(const Transform& orig);
    virtual ~Transform();
    motion_control::Pose antiTrasform();
    discrete_controller::Transform static TransformPose(motion_control::Pose pose);
    motion_control::Pose static antiTransformPose(discrete_controller::Transform state);
    motion_control::Velocity static control(discrete_controller::Command cmd, const motion_control::Pose *pose);
private:
    discrete_controller::Transform state;
};

#endif	/* TRANSFORM_H */

