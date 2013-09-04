/* 
 * File:   Unicycle.h
 * Author: raffaello
 *
 * Created on 02 September 2013, 15:35
 */

#ifndef UNICYCLE_H
#define	UNICYCLE_H

#include <motion_control/Pose.h>
#include <motion_control/Velocity.h>

class Unicycle {
public:
    Unicycle();
    Unicycle(motion_control::Pose pose);
    Unicycle(const Unicycle& orig);
    virtual ~Unicycle();
    void update();
    void setVelocity(motion_control::Velocity vel);
    motion_control::Pose getPose();
private:
    motion_control::Velocity velocity;
    motion_control::Pose pose;
};

#endif	/* UNICYCLE_H */

