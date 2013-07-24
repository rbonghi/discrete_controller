/* 
 * File:   discrete_node.cpp
 * Author: raffaello
 *
 * Created on July 24, 2013, 5:04 PM
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

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
  
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    ros::spinOnce();

    ROS_INFO("HELLO CONTROLLER");
    
    loop_rate.sleep();
  }

  return 0;
}

