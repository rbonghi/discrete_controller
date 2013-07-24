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

class Transform {
public:
    Transform(motion_control::Pose pose);
    Transform(const Transform& orig);
    virtual ~Transform();
    motion_control::Pose antiTrasform();
    motion_control::Pose static antiTrasform(discrete_controller::Transform state);
private:
    discrete_controller::Transform state;
};

#endif	/* TRANSFORM_H */

