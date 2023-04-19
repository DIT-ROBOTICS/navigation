#ifndef _NAVIGATION_MAIN_H_
#define _NAVIGATION_MAIN_H_

// ROS basic
#include "ros/ros.h"

// ROS msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>

// ROS srvs
#include "std_srvs/Empty.h"

// ROS param
#include <dynamic_reconfigure/server.h>

#include "navigation_main/navigation_main_paramConfig.h"

// Other lib
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

class Navigation_Main {
   public:
    Navigation_Main();
    ~Navigation_Main();

    void Init(ros::NodeHandle *nh_global, ros::NodeHandle *nh_local, std::string node_name);
    double GetUpdateFrequency();

    void Loop();

   private:
    bool
    UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void SetDynamicReconfigure();

    void SetTimeout(geometry_msgs::Pose poseGoal);
    bool isTimeout();
    void FailToGoal();

    // Callback functions
    void Robot_Odom_type0_CB(const nav_msgs::Odometry::ConstPtr &msg);
    void Robot_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void Robot_Obs_Odom_type0_CB(const nav_msgs::Odometry::ConstPtr &msg);
    void Robot_Obs_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void Rival1_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg);
    void Rival2_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg);
    void PathTrackerCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msgs);
    void DockTrackerCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msgs);
    void RobotMissionState_CB(const std_msgs::Char::ConstPtr &msg);
    void MainMission_CB(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void DynamicParam_CB(const navigation_main::navigation_main_paramConfig &config, uint32_t level);
    void ResendGoal_CB(const ros::TimerEvent &event);

    // Other functions
    bool isCloseToOtherRobots();
    double Distance_Between_A_and_B(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);

    // NodeHandle
    ros::NodeHandle *nh_local_;
    ros::NodeHandle *nh_global_;

    // Subscriber
    ros::Subscriber robot_odom_sub_;
    ros::Subscriber robot_obs_odom_sub_;
    ros::Subscriber rival_odom_sub_[2];
    ros::Subscriber robot_path_tracker_cmd_vel_sub_;
    ros::Subscriber robot_dock_tracker_cmd_vel_sub_;
    ros::Subscriber robot_mission_state_sub_;
    ros::Subscriber main_mission_state_sub_;

    // Publisher
    ros::Publisher main_mission_state_pub_;
    ros::Publisher robot_path_tracker_goal_pub_;
    ros::Publisher robot_dock_tracker_goal_pub_;
    ros::Publisher robot_cmd_vel_pub_;

    // Server
    ros::ServiceServer param_srv_;

    // Parameters
    bool param_active_;
    bool param_publish_;
    bool param_update_params_;
    bool param_use_dynamic_reconfigure_;

    int param_resend_goal_time_;

    double param_update_frequency_;
    double param_timeout_;
    double param_timeout_a_;
    double param_timeout_b_;
    double param_timeout_min_;
    double param_timeout_max_;
    double param_resend_goal_frequency_;
    double param_stop_distance_;
    double param_odom_timeout_;

    std::string param_node_name_;
    std::string param_robot_odom_topic_;
    std::string param_robot_obs_odom_topic_;
    std::string param_rival_odom_topic_[2];
    std::string param_robot_mission_state_topic_;
    std::string param_robot_path_tracker_goal_topic_;
    std::string param_robot_dock_tracker_goal_topic_;
    std::string param_robot_cmd_vel_topic_;
    std::string param_robot_path_tracker_cmd_vel_topic_;
    std::string param_robot_dock_tracker_cmd_vel_topic_;
    std::string param_main_mission_topic_;
    std::string param_main_mission_state_topic_;

    // Variables
    bool is_mission_start_;
    bool is_reach_goal_;

    enum MISSION_TYPE {
        // Do mission
        PATH_TRACKER = 0,
        DOCK_TRACKER = 1,

        // Mission point was blocked
        RESEND_PATH_GOAL = 2,
        RESEND_DOCK_GOAL = 3,

        // Too close to other robots
        STOP_PATH = 4,
        STOP_DOCK = 5,

        IDLE = 6
    };
    MISSION_TYPE mission_status_;

    enum class ODOM_CALLBACK_TYPE {
        nav_msgs_Odometry = 0,
        geometry_msgs_PoseWithCovarianceStamped = 1
    };
    ODOM_CALLBACK_TYPE odom_type_;

    // Timeout
    ros::Time start_time_;
    ros::Timer resend_goal_timer_;
    int resend_goal_time_;

    ros::Time robot_obs_odom_time_;
    ros::Time rival_odom_time_[2];

    // Robot Odometry
    geometry_msgs::PoseStamped robot_goal_;
    geometry_msgs::Pose robot_odom_;
    geometry_msgs::Pose robot_obs_odom_;
    geometry_msgs::Pose rival_odom_[2];
    geometry_msgs::Twist robot_cmd_vel_;
};

extern Navigation_Main navigation_main_;

#endif