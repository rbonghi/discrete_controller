
#include "multi_rate_planner.h"
#include "transformation/Transform.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>

#include <boost/bind.hpp>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(multi_rate_planner, MultiRatePlanner, multi_rate_planner::MultiRatePlanner, nav_core::BaseGlobalPlanner)

// http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
// https://github.com/ros-planning/navigation/blob/jade-devel/navfn/src/navfn_ros.cpp
// https://github.com/ros-planning/navigation/blob/jade-devel/navfn/include/navfn/navfn_ros.h

namespace multi_rate_planner {

    MultiRatePlanner::MultiRatePlanner()
    : initialized_(false) //, costmap_ros_(NULL),  planner_(), allow_unknown_(true)
    {}

    MultiRatePlanner::MultiRatePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false) //, costmap_ros_(NULL),  planner_(), allow_unknown_(true)
    {
        //initialize the planner
        initialize(name, costmap_ros);
    }



    void MultiRatePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if(!initialized_) {

            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            //get the tf prefix
            ros::NodeHandle prefix_nh;
            tf_prefix_ = tf::getPrefixParam(prefix_nh);

            initialized_ = true;

            path_ = new PathPlotter(private_nh, 2, 1000);
            path_->setTime(10);
            path_->addRateCallback(&MultiRatePlanner::rate_fnc, this);
            path_->addMultiRateCallback(&MultiRatePlanner::multirate_fnc, this);

        } else {
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        }
    }

    bool MultiRatePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ) {
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        //clear the plan, just in case
        plan.clear();

        ROS_INFO_STREAM("INIT -> Start [" << start.pose.position.x << "," << start.pose.position.y << "]");
        ROS_INFO_STREAM("INIT -> Goal [" << goal.pose.position.x << "," << goal.pose.position.y << "]");


        path_->setGoal(&start, &goal, &plan, (AbstractTransform*) new Transform());
//        plan.push_back(start);
//        for (int i=0; i<20; i++){
//            geometry_msgs::PoseStamped new_goal = goal;
//            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
//            new_goal.pose.position.x = -2.5+(0.05*i);
//            new_goal.pose.position.y = -3.5+(0.05*i);

//            new_goal.pose.orientation.x = goal_quat.x();
//            new_goal.pose.orientation.y = goal_quat.y();
//            new_goal.pose.orientation.z = goal_quat.z();
//            new_goal.pose.orientation.w = goal_quat.w();
//            ROS_INFO_STREAM("[" << i << "] -> Goal [" << goal.pose.position.x << "," << goal.pose.position.y << "]");

//            plan.push_back(new_goal);
//        }
//        plan.push_back(goal);

        //plan.push_back(start);

        return true;
    }

    discrete_controller::Command MultiRatePlanner::rate_fnc(int& delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal)
    {
      Transform zr(pose_robot);
      //  ROS_INFO("zr [%f, %f, %f]", zr.state.z1, zr.state.z2, zr.state.z3);
      Transform zk(pose_goal);
      //  ROS_INFO("zk [z1: %f, z2: %f, z3: %f]", zk.state.z1, zk.state.z2, zk.state.z3);
      alpha_ = (zk - zr) / delta;
      //  ROS_INFO("alpha [%f, %f, %f]", alpha.state.z1, alpha.state.z2, alpha.state.z3);

      cmd.u1 = alpha_.state.z1;
      cmd.u2 = (-alpha_.state.z2 + (4 * alpha_.state.z3) / (delta * alpha_.state.z1) - (4 * zr.state.z2) / (delta));

      ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);

      return cmd;
    }

    discrete_controller::Command MultiRatePlanner::multirate_fnc(int& delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal) {
        double u21 = cmd.u2;
        cmd.u2 = (2 * alpha_.state.z2 - u21);
       ROS_INFO("u1: %f, u2: %f", cmd.u1, cmd.u2);
       return cmd;
    }

}
