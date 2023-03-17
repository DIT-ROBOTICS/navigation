#include "navigation_main/navigation_main.h"

Navigation_Main navigation_main_;

Navigation_Main::Navigation_Main() {
}

void Navigation_Main::Init(ros::NodeHandle *nh_global, ros::NodeHandle *nh_local) {
    this->nh_global_ = nh_global;
    this->nh_local_ = nh_local;
    robot_odom_.position.x = robot_odom_.position.y = 0.0;

    std_srvs::Empty empty;

    this->param_active_ = false;
    if (this->UpdateParams(empty.request, empty.response)) {
        ROS_INFO_STREAM("[navigation_main] : Success to init param.");
    } else {
        ROS_FATAL_STREAM("[navigation_main] : Fail to init param.");
    }
}

double Navigation_Main::GetUpdateFrequency() {
    return this->param_update_frequency_;
}

void Navigation_Main::Loop() {
    if (is_mission_start_) {
        if (isTimeout()) {
            is_mission_start_ = false;
            // Timeout routine ...
        }
    }
}

bool Navigation_Main::isTimeout() {
    return ((ros::Time::now() - start_time_).toSec() > param_timeout_);
}

bool Navigation_Main::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    bool prev_active = this->param_active_;

    if (this->nh_local_->param<bool>("active", param_active_, true)) {
        ROS_INFO_STREAM("[navigation_main] : active set to " << param_active_);
    }
    if (this->nh_local_->param<bool>("update_params", param_update_params_, false)) {
        ROS_INFO_STREAM("[navigation_main] : update params set to " << param_update_params_);
    }
    if (this->nh_local_->param<bool>("use_dynamic_reconfigure", param_use_dynamic_reconfigure_, false)) {
        ROS_INFO_STREAM("[navigation_main] : use_dynamic_reconfigure set to " << param_use_dynamic_reconfigure_);
    }
    if (this->nh_local_->param<double>("update_frequency", param_update_frequency_, 5.0)) {
        ROS_INFO_STREAM("[navigation_main] : update frequency set to " << param_update_frequency_);
    }
    if (this->nh_local_->param<std::string>("robot_odom_topic", param_robot_odom_topic_, "/robot1/odom")) {
        ROS_INFO_STREAM("[navigation_main] : Subscribe topic " << param_robot_odom_topic_);
    }
    if (this->nh_local_->param<std::string>("robot_mission_state_topic", param_robot_mission_state_topic_, "/robot1/mission_state")) {
        ROS_INFO_STREAM("[navigation_main] : Subscribe topic " << param_robot_mission_state_topic_);
    }

    if (param_active_ != prev_active) {
        if (param_active_) {
            ROS_INFO_STREAM("[navigation_main] : active node.");

            this->robot_odom_sub_ = this->nh_global_->subscribe(param_robot_odom_topic_, 100, &Navigation_Main::Pose_Callback, this);
            this->robot_mission_state_sub_ = this->nh_global_->subscribe(param_robot_mission_state_topic_, 100, &Navigation_Main::RobotMissionState_Callback, this);

            if (this->param_update_params_) {
                this->param_srv_ = nh_local_->advertiseService("params", &Navigation_Main::UpdateParams, this);
            }

            if (this->param_use_dynamic_reconfigure_) {
                this->SetDynamicReconfigure();
            }
        } else {
            this->robot_odom_sub_.shutdown();
            this->robot_mission_state_sub_.shutdown();

            if (this->param_update_params_) {
                this->param_srv_.shutdown();
            }
        }
    }

    return true;
}

void Navigation_Main::SetDynamicReconfigure() {
    static dynamic_reconfigure::Server<navigation_main::navigation_main_paramConfig> dynamic_param_srv_;

    dynamic_reconfigure::Server<navigation_main::navigation_main_paramConfig>::CallbackType callback;

    callback = boost::bind(&Navigation_Main::DynamicParam_Callback, this, _1, _2);

    // Set callback function to param server
    dynamic_param_srv_.setCallback(callback);
}

double Navigation_Main::Distance_Between_A_and_B(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB) {
    return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2));
}

// Callback function
void Navigation_Main::Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    robot_odom_ = msg->pose;
}

void Navigation_Main::RobotMissionState_Callback(const std_msgs::Bool::ConstPtr &msg) {
    this->is_reach_goal_ = msg->data;
}

void Navigation_Main::DynamicParam_Callback(navigation_main::navigation_main_paramConfig &config, uint32_t level) {
    if (param_active_ != config.active) {
        this->param_active_ = config.active;
        ROS_INFO_STREAM("[navigation_main] : active set to " << param_active_);
    }
}