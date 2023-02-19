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

    // read YAML parameter
    nh.param("update_frequency", update_frequency_, 10.0);

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
    if (!enabled_)
        return;

    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

    // for (int i = 0; i < obstacle_pos_.size(); i++) {
    //     double mark_x = obstacle_pos_[i].get_x();
    //     double mark_y = obstacle_pos_[i].get_y();
    //     unsigned int mx;
    //     unsigned int my;
    //     if (worldToMap(mark_x, mark_y, mx, my)) {
    //         setCost(mx, my, LETHAL_OBSTACLE);
    //     }
    //     // std::cout << mx << " " << my << std::endl;

    //     *min_x = std::min(*min_x, mark_x);
    //     *min_y = std::min(*min_y, mark_y);
    //     *max_x = std::max(*max_x, mark_x);
    //     *max_y = std::max(*max_y, mark_y);
    // }
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    // if (!enabled_)
    //   return;

    // for (int j = min_j; j < max_j; j++)
    // {
    //   for (int i = min_i; i < max_i; i++)
    //   {
    //     int index = getIndex(i, j);
    //     if (costmap_[index] == NO_INFORMATION)
    //       continue;
    //     master_grid.setCost(i, j, costmap_[index]);
    //   }
    // }

    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
    // updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
    // updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace path_layer_namespace