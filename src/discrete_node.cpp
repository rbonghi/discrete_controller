/* 
 * File:   discrete_node.cpp
 * Author: raffaello
 *
 * Created on July 24, 2013, 5:04 PM
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "MultiRate.h"

#include <cstdlib>

using namespace std;

/*
 * 
 */
int main(int argc, char** argv)
{
  //Init the serial_motion_node
  ros::init(argc, argv, "discrete_node");
  ros::NodeHandle nh;

  ros::Timer timer;
  MultiRate* rate = new MultiRate(nh, 1, 2);

  ROS_INFO("Start controller");
  ros::spin();
  ROS_INFO("EXIT");

  return 0;
}

