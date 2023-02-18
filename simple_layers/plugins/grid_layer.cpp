#include <simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
#include "simple_layers/grid_layer.h"

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace simple_layer_namespace
{
Obstacle::Obstacle(double x, double y, Obstacle_type obstacle_type, std::string source_type, ros::Time t)
{
  x_ = x;
  y_ = y;
  obstacle_type_ = obstacle_type;
  source_type_ = source_type;
  stamp_ = t;

}

GridLayer::GridLayer() 
{
}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  sub_ = g_nh_.subscribe("obstacle_position_array", 10, &GridLayer::obsCallback, this);

  // read YAML parameter
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
  nh.param("tolerance/sample", tolerance_sample_, 0.1);
  nh.param("tolerance/rival", tolerance_rival_, 0.1);

  nh.getParam("filter/enabled", filter_enabled_);
  nh.getParam("filter/quiescence", filter_quiescence_);
  nh.getParam("filter/beta", filter_beta_);
  nh.getParam("filter/fixed_point_remove_", fixed_point_remove_);
  nh.getParam("filter/threshold_time_", threshold_time_);
  
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
  // get the sersor name and obstacle type. format ex: sample_camera 
  std::string temp = poses.header.frame_id;
  int idx = temp.find("_");
  std::string sensor_name;
  std::string obs_type_temp;
  if(idx!=std::string::npos){
    obs_type_temp = temp.substr(0,idx);
    sensor_name = temp.substr(idx+1);
  }else{
    ROS_WARN("Grid_layer: callback frame_id %s format is wrong", temp.c_str());
    return;
  }
  Obstacle_type obstacle_type;
  if(obs_type_temp == "sample")
    obstacle_type = Obstacle_type::sample;
  else if(obs_type_temp == "rival")
    obstacle_type = Obstacle_type::rival;

  if(!ifAddToLayer(sensor_name)) return;

  unsigned int obstacle_num = poses.poses.size();
  ROS_INFO("GridLayer: Received %d Obstacles from Sources: %s", obstacle_num, sensor_name.c_str());
  
  for (int i = 0; i < obstacle_num; i++){
    Obstacle obs(poses.poses[i].position.x, poses.poses[i].position.y, obstacle_type, sensor_name, poses.header.stamp);
    
    if(filter_enabled_ && fixed_point_remove_){
      for (auto j = obstacle_pos_.begin(); j != obstacle_pos_.end(); ++j){
        double diff = ros::Time::now().toSec() - j->get_time().toSec();
        if( diff > threshold_time_) obstacle_pos_.erase(j);
      }
    }
    if(filter_enabled_ && filter_quiescence_){
      static std::vector<int> adjacent_obs_num;  
      adjacent_obs_num.clear();
      adjacent_obs_num = ifExists(obs);
    
      if (adjacent_obs_num.size() != 0){
        for (int j=0; j<adjacent_obs_num.size(); j++){
          obs = lowpassFilter(obs, obstacle_pos_, adjacent_obs_num);
          obstacle_pos_.erase(obstacle_pos_.begin() + adjacent_obs_num[j] - j);
        }
      }
    }
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
  if (obstacle_pos_.size() == 0)
    return;

  resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

  for(int i=0; i<obstacle_pos_.size(); i++){
    double mark_x = obstacle_pos_[i].get_x();
    double mark_y = obstacle_pos_[i].get_y();
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my)){
      setCost(mx, my, LETHAL_OBSTACLE);
    }
    // std::cout << mx << " " << my << std::endl;
    
    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
  }

}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
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

bool GridLayer::ifAddToLayer(std::string observation_source_type)
{
  for(int i=0; i<observation_sources_.size(); i++)
  {
    if(observation_sources_[i] == observation_source_type)
      return true;
  }
  return false;
}

std::vector<int> GridLayer::ifExists(Obstacle obs)
{
  std::vector<int> idxs;
  for(int i=0; i<obstacle_pos_.size(); i++)
  {
    double dist = sqrt(pow(obs.get_x() - obstacle_pos_[i].get_x(), 2) + pow(obs.get_y() - obstacle_pos_[i].get_y(), 2));
    switch(obs.get_obstacle_type()){
      case Obstacle_type::sample:
        if(dist < tolerance_sample_)
          idxs.push_back(i);
        break;

      case Obstacle_type::rival:
        if(dist < tolerance_rival_)
          idxs.push_back(i);
        break;
    }
  }

  return idxs;
}

  Obstacle GridLayer::lowpassFilter(Obstacle obs_new, std::vector<Obstacle> obs, std::vector<int> idxs)
  {
    int idx_latest = idxs[0];
    for(int i=0; i<idxs.size()-1; i++)
    {
      if(obs[idxs[i]].get_time()-obs_new.get_time() > obs[idxs[i+1]].get_time()-obs_new.get_time())  
        idx_latest = i+1;
    }
    // std::cout << filter_beta_ << std::endl;
    obs_new.set_x( filter_beta_*obs_new.get_x() + (1-filter_beta_)*obs[idx_latest].get_x());
    obs_new.set_y( filter_beta_*obs_new.get_y() + (1-filter_beta_)*obs[idx_latest].get_y());
    return obs_new;
  }
} // end namespace