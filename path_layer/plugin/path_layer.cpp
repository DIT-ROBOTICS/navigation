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
    RivalPath_Sub = Global_nh.subscribe("RivalPose", 1000, &PathLayer::RivalPath_CB, this);

    // read YAML parameter
    nh.param("update_frequency", update_frequency_, 10.0);
    nh.param("enabled", enabled_, true);

    isRivalPath = false;

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
    if (!enabled_ || !isRivalPath)
        return;

    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

    double mark_x = RivalPath.pose.position.x;
    double mark_y = RivalPath.pose.position.y;

    unsigned int mx;
    unsigned int my;

    // for (int i = 0; i < 1; i++) {
    //     if (worldToMap(mark_x, mark_y + i / 10.0, mx, my)) {
    //     }
    // }

    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);

    // Debug
    // printf("mx : %3d my : %3d\n", mx, my);

    printf("Time : %.2f\n", ros::Time::now().toSec());
    printf("min X : %.2f Y : %.2f\n", *min_x, *min_y);
    printf("max X : %.2f Y : %.2f\n", *max_x, *max_y);
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_ || !isRivalPath)
        return;

    double mark_x = RivalPath.pose.position.x;
    double mark_y = RivalPath.pose.position.y;

    unsigned int mx;
    unsigned int my;

    if (worldToMap(mark_x, mark_y, mx, my)) {
        setCost(mx, my, LETHAL_OBSTACLE);
    }

    // Load the costmap_ to master_grid
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

void PathLayer::RivalPath_CB(const geometry_msgs::PoseStamped& Pose) {
    this->RivalPath = Pose;
    isRivalPath = true;
}

}  // namespace path_layer_namespace