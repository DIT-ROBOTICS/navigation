// plugin
#include <pluginlib/class_list_macros.h>

// include
#include "path_layer/path_layer.h"

PLUGINLIB_EXPORT_CLASS(path_layer_namespace::PathLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace path_layer_namespace {

PathLayer::PathLayer() {
}

void PathLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);

    std::string RobotPath_CB_TopicName;

    // read YAML parameter
    nh.param("update_frequency", update_frequency_, 10.0);
    nh.param("enabled", enabled_, true);
    nh.param("RobotPathTimeout", RobotPathTimeout, 1.0);
    nh.param("RobotPredictLength", RobotPredictLength, 1);
    nh.param<std::string>("Topic", RobotPath_CB_TopicName, "/move_base/GlobalPlanner/plan");

    RobotPath_Sub = nh.subscribe(RobotPath_CB_TopicName, 1000, &PathLayer::RobotPath_CB, this);

    isRobotPath = false;
    RobotPathLastTime = ros::Time::now();

    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    // Register layer
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&PathLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void PathLayer::matchSize() {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
              master->getResolution(),
              master->getOriginX(), master->getOriginY());
}

void PathLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {
    enabled_ = config.enabled;
}

void PathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y, double* max_x, double* max_y) {
    if (!enabled_ || !isRobotPath)
        return;

    /* Uncomment the code below to enable RobotPath's Timeout. */
    // if (ros::Time::now().toSec() - RobotPathLastTime.toSec() > RobotPathTimeout) {
    //     isRobotPath = false;
    //     return;
    // }

    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

    for (int i = 0; i < RobotPredictLength; i++) {
        if (RobotPath.poses.size() <= i) {
            break;
        }

        double mark_x = RobotPath.poses[i].pose.position.x;
        double mark_y = RobotPath.poses[i].pose.position.y;
        unsigned int mx;
        unsigned int my;

        if (worldToMap(mark_x, mark_y, mx, my)) {
            *min_x = std::min(*min_x, mark_x);
            *min_y = std::min(*min_y, mark_y);
            *max_x = std::max(*max_x, mark_x);
            *max_y = std::max(*max_y, mark_y);
            setCost(mx, my, LETHAL_OBSTACLE);
        }
    }

    *min_x -= 2;
    *min_y -= 2;
    *max_x += 2;
    *max_y += 2;

    // Debug
    // printf("mx : %3d my : %3d\n", mx, my);
    // printf("Time : %.2f\n", ros::Time::now().toSec());
    // printf("min X : %.2f Y : %.2f\n", *min_x, *min_y);
    // printf("max X : %.2f Y : %.2f\n", *max_x, *max_y);
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_ || !isRobotPath)
        return;

    boost::unique_lock<mutex_t> lock(*(getMutex()));
    // updateWithAddition(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
    updateWithOverwrite(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());

    // Load the costmap_ to master_grid
    // updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void PathLayer::RobotPath_CB(const nav_msgs::Path& Path) {
    this->RobotPath = Path;
    isRobotPath = true;
    RobotPathLastTime = ros::Time::now();
}

}  // namespace path_layer_namespace