#ifndef _NAVIGATION_MAIN_H_
#define _NAVIGATION_MAIN_H_

// ROS basic
#include "ros/ros.h"

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

// Other lib
#include <cstdlib>

class NAVIGATION_MAIN {
   public:
    NAVIGATION_MAIN();

    void Init(ros::NodeHandle *nh_global, ros::NodeHandle *nh_local);

   private:
    ros::NodeHandle *nh_local;
    ros::NodeHandle *nh_global;
};

extern NAVIGATION_MAIN nav_main;

#endif