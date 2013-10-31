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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <discrete_controller/Command.h>
#include <std_srvs/Empty.h>

const std::string goal_string = "goal";
const std::string path_string = "path";
const std::string command_string = "command";
const std::string desidered_unicycle_string = "desidered_unicycle";
const std::string pose_array_string = "pose_step";
const std::string control_stop_string = "/control/emergency";

class PathPlotter {
public:
    typedef discrete_controller::Command(* ActionType) (int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    typedef serial_bridge::Velocity(* ControllerType) (ros::NodeHandle nh, serial_bridge::Velocity velocityd, geometry_msgs::PoseStamped posed, nav_msgs::Odometry pose_robot);
    PathPlotter(const ros::NodeHandle& nh, std::string name, int rate, int length);
    PathPlotter(const PathPlotter& orig);
    virtual ~PathPlotter();

    void setTime(int time);
    unsigned int getTime();
    unsigned int getMultiRate();
    void setActionStop(ActionType action);
    void setActionRate(ActionType action);
    void setActionMultiRate(ActionType action);

    void setPathController(ControllerType controller);
    
    void startController(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal, AbstractTransform* transform, std::string robot, std::string odometry, std::string velocity);
    void setGoal(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal, AbstractTransform* transform);
private:
    std::string name_;  //Name param and topic control
    //step trajectory
    int length_step_, counter_step_pose_array_, step_pose_array_;
    geometry_msgs::PoseArray array_step_;
    // Actions for multirate
    ActionType *pActions;
    ActionType actionRate_, actionStop_;
    unsigned int counter_actions_;

    ros::NodeHandle nh_; //ROS Node controller
    //Publisher for print goal and path robot
    ros::Publisher pub_goal_, pub_path_;
    ros::Publisher pub_multirate_, pub_desidered_unicycle_;      // Pubblisher command chained form
    int length_; //Length array for path
    nav_msgs::Path path_;
    Unicycle* unicycle_; //Unicycle for plot trajectory

    bool stop_; //Stop timer
    ros::Timer timer_; //Timer for esecution coomand robot
    int time_, multirate_, counter_; //Time for control plan and multi rate
    const geometry_msgs::PoseStamped* pose_robot_controller_;
    const geometry_msgs::PoseStamped* pose_goal_controller_;
    AbstractTransform* transform_controller_;
    discrete_controller::Command cmd_controller_; //Command to drive robot
    ros::Subscriber sub_odometry_;
    ros::Publisher pub_path_control_;
    ros::Publisher pub_array_step_; //Array pose robots
    ControllerType controller_;
    
    ros::ServiceServer control_stop_srv_;

    discrete_controller::Command stop(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    discrete_controller::Command rateStep(const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    discrete_controller::Command multiRateStep(int step, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    void draw_path(AbstractTransform* transform, discrete_controller::Command cmd, int counter, int step, double update);

    void odometry_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    serial_bridge::Velocity path_controller (serial_bridge::Velocity velocityd, geometry_msgs::PoseStamped posed, nav_msgs::Odometry pose_robot);
    
    bool control_stop_Callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
};

#endif	/* PATHPLOTTER_H */

