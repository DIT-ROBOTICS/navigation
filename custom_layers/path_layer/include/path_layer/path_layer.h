#ifndef PATH_LAYER_H_
#define PATH_LAYER_H_

// ros
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros/time.h>

// costmap
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

// msgs
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

// other
#include <string>

namespace path_layer_namespace {

enum class Robot_type {
    robot1,
    robot2
};

// public costmap_2d::CostmapLayer
// public costmap_2d::Layer, public costmap_2d::Costmap2D
class PathLayer : public costmap_2d::CostmapLayer {
   public:
    PathLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    bool isDiscretized() {
        return true;
    }

    virtual void matchSize();

    void RivalPath_CB(const nav_msgs::Path& Path);

   private:
    void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;

    ros::NodeHandle Global_nh;
    ros::Subscriber RivalPath_Sub;

    std::string RivalPath_CB_TopicName;

    ros::Time RivalPathLastTime;
    double RivalPathTimeout;

    int RivalPredictLength;

    bool isRivalPath = false;
    nav_msgs::Path RivalPath;

    double update_frequency_;
    std::vector<std::string> observation_sources_;
};

}  // namespace path_layer_namespace

#endif