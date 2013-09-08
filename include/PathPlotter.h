/* 
 * File:   PathPlotter.h
 * Author: raffaello
 *
 * Created on 08 September 2013, 17:04
 */

#ifndef PATHPLOTTER_H
#define	PATHPLOTTER_H

#include "Unicycle.h"
#include "transformation/AbstractTransform.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <discrete_controller/Command.h>

const std::string goal_string = "goal";
const std::string path_string = "path";

class PathPlotter {
public:
    typedef discrete_controller::Command(* ActionType) (int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    PathPlotter(const ros::NodeHandle& nh, std::string name, int rate, int length);
    PathPlotter(const PathPlotter& orig);
    virtual ~PathPlotter();


    void setTime(int time);
    unsigned int getTime();
    unsigned int getMultiRate();
    void setActionStop(ActionType action);
    void setActionRate(ActionType action);
    void setActionMultiRate(ActionType action);
    
    void setGoal(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal, AbstractTransform* transform);
private:
    // Actions for multirate
    ActionType *pActions;
    ActionType actionRate_, actionStop_;
    unsigned int counter_actions_;

    ros::NodeHandle nh_; //ROS Node controller
    //Publisher for print goal and path robot
    ros::Publisher pub_goal_, pub_path_;
    int length_; //Length array for path
    nav_msgs::Path path_;
    Unicycle* unicycle_; //Unicycle for plot trajectory
    discrete_controller::Command cmd_; //Command to drive robot

    int time_, multirate_; //Time for control plan and multi rate

    discrete_controller::Command rateStep(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    discrete_controller::Command multiRateStep(int step, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    void draw_path(AbstractTransform* transform, int counter, int step);
};

#endif	/* PATHPLOTTER_H */

