/** include the libraries you need in your planner here */
 /** for global path planner interface */

#ifndef MULTI_RATE_PLANNER_H_
#define MULTI_RATE_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "PathPlotter.h"

 using std::string;

namespace multi_rate_planner {

class MultiRatePlanner : public nav_core::BaseGlobalPlanner {
public:

    MultiRatePlanner();
    MultiRatePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan
                  );

protected:

    /**
    * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
    */
    costmap_2d::Costmap2DROS* costmap_ros_;
    bool initialized_;
    ros::Publisher plan_pub_;

    PathPlotter* path_;
    //boost::shared_ptr<NavFn> planner_;

    //pcl_ros::Publisher<PotarrPoint> potarr_pub_;
    //bool initialized_, allow_unknown_, visualize_potential_;

private:
    std::string tf_prefix_;

    discrete_controller::Command cmd;

    discrete_controller::Command rate_fnc(int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
    discrete_controller::Command multirate_fnc(int delta, const geometry_msgs::PoseStamped* pose_robot, const geometry_msgs::PoseStamped* pose_goal);
};

}

#endif
