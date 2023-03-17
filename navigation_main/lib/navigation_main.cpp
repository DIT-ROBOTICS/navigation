#include "navigation_main/navigation_main.h"

NAVIGATION_MAIN nav_main;

NAVIGATION_MAIN::NAVIGATION_MAIN() {
}

void NAVIGATION_MAIN::Init(ros::NodeHandle *nh_global, ros::NodeHandle *nh_local) {
    this->nh_global = nh_global;
    this->nh_local = nh_local;
}