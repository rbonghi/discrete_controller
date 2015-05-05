
#include "multi_rate_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>


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

    }

}
