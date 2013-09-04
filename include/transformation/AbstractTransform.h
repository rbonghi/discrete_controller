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

class AbstractTransform {
public:
    virtual motion_control::Velocity control(discrete_controller::Command cmd){};
private:
    virtual discrete_controller::Transform transformPose(const motion_control::Pose *pose){};
};

#endif	/* ABSTRACTTRANSFORM_H */

