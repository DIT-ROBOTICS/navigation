#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/PoseArray.h"
#include "ros/time.h"

namespace simple_layer_namespace
{
enum class Obstacle_type{
  sample,
  rival
};

class Obstacle
{
public:
  Obstacle();
  Obstacle(double x, double y, Obstacle_type obstacle_type, std::string source_type, ros::Time t);
  double get_x() const{
    return x_;
  }
  double get_y() const{
    return y_;
  }
  Obstacle_type get_obstacle_type() const{
    return obstacle_type_;
  }

private:
  double x_;
  double y_;
  double z_;
  double vx_;
  double vy_;
  double w;
  double yaw_;

  std::string source_type_;
  Obstacle_type obstacle_type_;
  ros::Time stamp_;

};

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GridLayer();

  /**
   * @brief callback function for adding obstacles
   * @param poses which type is geometry_msgs/PoseArray
   * poses.header.frame_id = which source_type is. ex: "camera", "tracker", "lidar"
   * poses.header.stamp = ros::Time::now()
   * poses.poses[??].x = position_x 
   * poses.poses[??].y = ...
   */
  void obsCallback(const geometry_msgs::PoseArray& poses);

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  std::vector<Obstacle> obstacle_pos_;
  ros::Subscriber sub_;
  ros::NodeHandle g_nh_;
  double update_frequency_;
  std::vector<std::string> observation_sources_;
  /** @brief tolerance that we think two obstacle are the same obstacle
   *  there are two type of them: sample and rival (other team's robot)
   */
  double tolerance_sample_;
  double tolerance_rival_;


  /**
   * @brief check whether the observation_source_type is used
   * @param observation_source_type the source_type to be checked
   */
  bool ifAddToLayer(std::string observation_source_type);

  /**
   * @brief check whether the obstacle exists
   * @param obs the obstacle to be checked
   * @return if exists return the index of the obstacle that same with the argument obstacle, if does not exist return empty vector
   */
  std::vector<int> ifExists(Obstacle obs);

  Obstacle lowPassFilter(std::vector<Obstacle> obs, std::vector<int> idxs);
};
}
#endif