#include <simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{
Obstacle::Obstacle(double x, double y, std::string type, ros::Time t)
{
  x_ = x;
  y_ = y;
  source_type_ = type;
  stamp_ = t;

}


GridLayer::GridLayer() 
{
}



void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  sub_ = g_nh_.subscribe("obstacle_position_array", 10, &GridLayer::obsCallback, this);

  bool observation_source_camera;
  bool observation_source_tracker;
  bool observation_source_lidar;
  nh.param("observation_sources/camera", observation_source_camera, false);
  nh.param("observation_sources/tracker", observation_source_tracker, false);
  nh.param("observation_sources/lidar", observation_source_lidar, false);
  if(observation_source_camera)
    observation_sources_.push_back("camera");
  if(observation_source_tracker)
    observation_sources_.push_back("tracker");
  if(observation_source_lidar)
    observation_sources_.push_back("lidar");
  std::string s = "";
  for(int i = 0; i < observation_sources_.size(); i++){
    s += observation_sources_[i];
    if(i < observation_sources_.size() - 1)
      s += ",";
  }
  ROS_INFO("Grid_layer: obseration_sources: %s", s.c_str());

  nh.param("update_frequency", update_frequency_, 10.0);

  obstacle_num_ = 0;



  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void GridLayer::obsCallback(const geometry_msgs::PoseArray& poses)
{
  if(!ifAddToLayer(poses.header.frame_id)) return;

  obstacle_num_ = poses.poses.size();
  ROS_INFO("GridLayer: Received %d Obstacles from Sources: %s", obstacle_num_, poses.header.frame_id.c_str());
  
  for(int i=0; i<obstacle_num_; i++)
  {
    Obstacle obs(poses.poses[i].position.x, poses.poses[i].position.y, poses.header.frame_id ,poses.header.stamp);
    obstacle_pos_.push_back(obs);
  }
  

}

void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  if (obstacle_num_ == 0)
    return;

  // std::cout << obstacle_num_ << std::endl;
  for(int i=0; i<obstacle_num_; i++){
    double mark_x = obstacle_pos_[i].get_x();
    double mark_y = obstacle_pos_[i].get_y();
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my)){
      setCost(mx, my, LETHAL_OBSTACLE);
    }
    std::cout << mx << " " << my << std::endl;
    
    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
  }

}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}


bool GridLayer::ifAddToLayer(std::string observation_source_type)
{
  for(int i=0; i<observation_sources_.size(); i++)
  {
    if(observation_sources_[i] == observation_source_type)
      return true;
  }
  return false;
}




} // end namespace