#include <simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/PoseArray.h"



PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace simple_layer_namespace
{

GridLayer::GridLayer()
{
  obstacle_updated_ = false;
  obstacle_num_ = 0;
  state_ = 0;
  step_ = 0;
  
}

void GridLayer::obs_callback(const geometry_msgs::PoseArray& poses)
{
  ROS_INFO("Obstacle Updated");
  // double mx = pose.pose.position.x;
  // double my = pose.pose.position.y;
  // obstacle_pos_[obstacle_num_].push_back(mx);
  // obstacle_pos_[obstacle_num_].push_back(my);
  // obstacle_num_++;
  obstacle_updated_ = true;
  

}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  sub_ = g_nh_.subscribe("obstacle_position_array", 10, &GridLayer::obs_callback, this);

  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void GridLayer::activate()
{
  onInitialize();
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

  // double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  // unsigned int mx;
  // unsigned int my;
  
  // if(worldToMap(mark_x, mark_y, mx, my)){
  //   setCost(mx, my, LETHAL_OBSTACLE);
  // }
  // 
  // *min_x = std::min(*min_x, mark_x);
  // *min_y = std::min(*min_y, mark_y);
  // *max_x = std::max(*max_x, mark_x);
  // *max_y = std::max(*max_y, mark_y);

  // ros::NodeHandle nh("temp");
  // ros::Rate r(5);

  
  // my_queue.callAvailable(ros::WallDuration());
  // while(!obstacle_updated_ && nh.ok()){
  //   ros::spinOnce();
  //   r.sleep();
  //   std::cout << "hello"<< std::endl;
  // }
  // ros::spinOnce();

  double extra_obstacle[3][2]={{0.5,0.5},{1,0.5},{1.5,0.5}};
  double obstacle1[2] = {1,1};
  double obstacle2[2] = {2,1};
  int res = 20;
  double mul = (double)step_ / res;

  double mark_x, mark_y;
  mark_x = obstacle1[0] + (obstacle2[0] - obstacle1[0]) * mul;
  mark_y = obstacle1[1] + (obstacle2[1] - obstacle1[1]) * mul;
  if(state_ == 0){
    if(step_ < res){
      step_++;
    }else if(step_ == res){
      step_--;
      state_ = 1;
    }
  }else if(state_ == 1){
    if(step_ > 0){
      step_--;
    }else if(step_ == 0){
      step_++;
      state_ = 0;
    }
  }

  unsigned int mx;
  unsigned int my;
  unsigned int mx_bef;
  unsigned int my_bef;
  if(worldToMap(mark_x, mark_y, mx, my)){
    setCost(mx, my, LETHAL_OBSTACLE);
    
    worldToMap(mark_x_bef, mark_y_bef, mx_bef, my_bef);
    setCost(mx_bef, my_bef, FREE_SPACE);
  }
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
  
  mark_x_bef = mark_x;
  mark_y_bef = mark_y;

  
  // std::printf("mark_x:%lf, mark_y:%lf, mx:%d, my:%d \n", mark_x, mark_y, mx, my);


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

} // end namespace