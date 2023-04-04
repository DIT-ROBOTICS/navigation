#include "navigation_main/navigation_main.h"

Navigation_Main navigation_main_;

Navigation_Main::Navigation_Main() {
}

Navigation_Main::~Navigation_Main() {
    if (param_active_) {
        this->robot_odom_sub_.shutdown();
        this->robot_mission_state_sub_.shutdown();
        this->robot_path_tracker_goal_pub_.shutdown();
        this->robot_dock_tracker_goal_pub_.shutdown();
        this->robot_cmd_vel_pub_.shutdown();
        this->robot_path_tracker_cmd_vel_sub_.shutdown();
        this->robot_dock_tracker_cmd_vel_sub_.shutdown();
        this->main_mission_state_sub_.shutdown();
        this->main_mission_state_pub_.shutdown();

        if (this->param_update_params_) {
            this->param_srv_.shutdown();
        }
    }
}

void Navigation_Main::Init(ros::NodeHandle *nh_global, ros::NodeHandle *nh_local, std::string node_name) {
    this->nh_global_ = nh_global;
    this->nh_local_ = nh_local;
    this->param_node_name_ = node_name;
    robot_odom_.position.x = robot_odom_.position.y = 0.0;
    this->mission_status_ = mission_type::IDLE;

    std_srvs::Empty empty;

    this->param_active_ = false;
    if (this->UpdateParams(empty.request, empty.response)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Success to init param.");
    } else {
        ROS_FATAL_STREAM("[" << param_node_name_ << "] : Fail to init param.");
    }
}

double Navigation_Main::GetUpdateFrequency() {
    return this->param_update_frequency_;
}

void Navigation_Main::Loop() {
    if (param_active_ && is_mission_start_ && isTimeout()) {
        is_mission_start_ = false;

        std_msgs::Bool temp;
        temp.data = false;
        main_mission_state_pub_.publish(temp);

        this->mission_status_ = mission_type::IDLE;
        this->robot_cmd_vel_.linear.x = this->robot_cmd_vel_.linear.y = this->robot_cmd_vel_.angular.z = 0.0;
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Timeout)");
        ROS_ERROR_STREAM("[" << param_node_name_ << "] : Mission Failed.");
    }

    this->robot_cmd_vel_pub_.publish(this->robot_cmd_vel_);
}

bool Navigation_Main::isTimeout() {
    return ((ros::Time::now() - start_time_).toSec() > param_timeout_);
}

bool Navigation_Main::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    bool prev_active = this->param_active_;

    int temp_odom_type_;

    if (this->nh_local_->param<bool>("active", param_active_, true)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : active set to " << param_active_);
    }
    if (this->nh_local_->param<bool>("update_params", param_update_params_, false)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : update params set to " << param_update_params_);
    }
    if (this->nh_local_->param<bool>("use_dynamic_reconfigure", param_use_dynamic_reconfigure_, false)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : use_dynamic_reconfigure set to " << param_use_dynamic_reconfigure_);
    }
    if (this->nh_local_->param<int>("odom_type", temp_odom_type_, 0)) {
        this->odom_type_ = (temp_odom_type_ == 0) ? odom_callback_type::nav_msgs_Odometry : odom_callback_type::geometry_msgs_PoseWithCovarianceStamped;
        ROS_INFO_STREAM("[" << param_node_name_ << "] : odom type set to " << temp_odom_type_);
    }
    if (this->nh_local_->param<double>("update_frequency", param_update_frequency_, 5.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : update frequency set to " << param_update_frequency_);
    }
    if (this->nh_local_->param<double>("timeout_a", param_timeout_a_, 5.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : timeout_a set to " << param_timeout_a_);
    }
    if (this->nh_local_->param<double>("timeout_b", param_timeout_b_, 5.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : timeout_b set to " << param_timeout_b_);
    }
    if (this->nh_local_->param<double>("min_timeout", param_timeout_min_, 5.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : timeout_min set to " << param_timeout_min_);
    }
    if (this->nh_local_->param<double>("max_timeout", param_timeout_max_, 5.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : timeout_max set to " << param_timeout_max_);
    }
    if (this->nh_local_->param<std::string>("robot_odom_topic", param_robot_odom_topic_, "/robot1/odom")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_robot_odom_topic_);
    }
    if (this->nh_local_->param<std::string>("robot_mission_state_topic", param_robot_mission_state_topic_, "/robot1/finishornot")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_robot_mission_state_topic_);
    }
    if (this->nh_local_->param<std::string>("robot_path_tracker_goal_topic", param_robot_path_tracker_goal_topic_, "/robot1/path_tracker_goal")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Publish topic " << param_robot_path_tracker_goal_topic_);
    }
    if (this->nh_local_->param<std::string>("robot_dock_tracker_goal_topic", param_robot_dock_tracker_goal_topic_, "/robot1/dock_tracker_goal")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Publish topic " << param_robot_dock_tracker_goal_topic_);
    }
    if (this->nh_local_->param<std::string>("robot_cmd_vel_topic", param_robot_cmd_vel_topic_, "/robot1/raw_cmd_vel")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Publish topic " << param_robot_cmd_vel_topic_);
    }
    if (this->nh_local_->param<std::string>("robot_path_tracker_cmd_vel_topic", param_robot_path_tracker_cmd_vel_topic_, "/robot1/path_tracker_cmd_vel")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_robot_path_tracker_cmd_vel_topic_);
    }
    if (this->nh_local_->param<std::string>("robot_dock_tracker_cmd_vel_topic", param_robot_dock_tracker_cmd_vel_topic_, "/robot1/dock_tracker_cmd_vel")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_robot_dock_tracker_cmd_vel_topic_);
    }
    if (this->nh_local_->param<std::string>("main_mission_topic", param_main_mission_topic_, "/navigation_main_1/mission")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_main_mission_topic_);
    }
    if (this->nh_local_->param<std::string>("main_mission_state_topic", param_main_mission_state_topic_, "/navigation_main_1/robot_goal")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Publish topic " << param_main_mission_state_topic_);
    }

    if (param_active_ != prev_active) {
        if (param_active_) {
            ROS_INFO_STREAM("[" << param_node_name_ << "] : active node.");

            // Subscribe
            switch (odom_type_) {
                case odom_callback_type::nav_msgs_Odometry:
                    this->robot_odom_sub_ = this->nh_global_->subscribe(param_robot_odom_topic_, 100, &Navigation_Main::Odom_type0_Callback, this);
                    break;
                case odom_callback_type::geometry_msgs_PoseWithCovarianceStamped:
                    this->robot_odom_sub_ = this->nh_global_->subscribe(param_robot_odom_topic_, 100, &Navigation_Main::Odom_type1_Callback, this);
                    break;
            }
            this->robot_mission_state_sub_ = this->nh_global_->subscribe(param_robot_mission_state_topic_, 100, &Navigation_Main::RobotMissionState_Callback, this);
            this->main_mission_state_sub_ = this->nh_global_->subscribe(param_main_mission_topic_, 100, &Navigation_Main::MainMission_Callback, this);
            this->robot_path_tracker_cmd_vel_sub_ = this->nh_global_->subscribe(param_robot_path_tracker_cmd_vel_topic_, 100, &Navigation_Main::PathTrackerCmdVel_Callback, this);
            this->robot_dock_tracker_cmd_vel_sub_ = this->nh_global_->subscribe(param_robot_dock_tracker_cmd_vel_topic_, 100, &Navigation_Main::DockTrackerCmdVel_Callback, this);

            // Publish
            this->robot_cmd_vel_pub_ = this->nh_global_->advertise<geometry_msgs::Twist>(param_robot_cmd_vel_topic_, 100);
            this->main_mission_state_pub_ = this->nh_global_->advertise<std_msgs::Bool>(param_main_mission_state_topic_, 100);
            this->robot_path_tracker_goal_pub_ = this->nh_global_->advertise<geometry_msgs::PoseStamped>(param_robot_path_tracker_goal_topic_, 100);
            this->robot_dock_tracker_goal_pub_ = this->nh_global_->advertise<geometry_msgs::PoseStamped>(param_robot_dock_tracker_goal_topic_, 100);

            if (this->param_update_params_) {
                this->param_srv_ = nh_local_->advertiseService("params", &Navigation_Main::UpdateParams, this);
            }

            if (this->param_use_dynamic_reconfigure_) {
                this->SetDynamicReconfigure();
            }
        } else {
            this->robot_odom_sub_.shutdown();
            this->robot_mission_state_sub_.shutdown();
            this->robot_path_tracker_goal_pub_.shutdown();
            this->robot_dock_tracker_goal_pub_.shutdown();
            this->robot_cmd_vel_pub_.shutdown();
            this->robot_path_tracker_cmd_vel_sub_.shutdown();
            this->robot_dock_tracker_cmd_vel_sub_.shutdown();
            this->main_mission_state_sub_.shutdown();
            this->main_mission_state_pub_.shutdown();

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

// Total Timeout = min_timeout < [(Distance to Goal) * (timeout_a) + (timeout_b)] < max_timeout
void Navigation_Main::SetTimeout(geometry_msgs::Pose poseGoal) {
    this->param_timeout_ = std::min(std::max(Distance_Between_A_and_B(poseGoal, robot_odom_) * param_timeout_a_ + param_timeout_b_, param_timeout_min_), param_timeout_max_);
    ROS_INFO_STREAM("[" << param_node_name_ << "] : set timeout to " << param_timeout_);
}

double Navigation_Main::Distance_Between_A_and_B(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB) {
    // ROS_INFO_STREAM("Distance : " << sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2)));
    return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2));
}

// Callback function
void Navigation_Main::Odom_type0_Callback(const nav_msgs::Odometry::ConstPtr &msg) {
    robot_odom_ = msg->pose.pose;
}
void Navigation_Main::Odom_type1_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_odom_ = msg->pose.pose;
}

void Navigation_Main::RobotMissionState_Callback(const std_msgs::Bool::ConstPtr &msg) {
    this->is_reach_goal_ = msg->data;

    if (param_active_ && is_mission_start_) {
        if (is_reach_goal_) {
            is_mission_start_ = false;

            std_msgs::Bool temp;
            temp.data = true;
            main_mission_state_pub_.publish(temp);

            this->mission_status_ = mission_type::IDLE;
            this->robot_cmd_vel_.linear.x = this->robot_cmd_vel_.linear.y = this->robot_cmd_vel_.angular.z = 0.0;
            ROS_INFO_STREAM("[" << param_node_name_ << "] : Mission end.");
        } else {
            ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Unreachable Goal)");

            // If there has some time remaining, retry to finish the mission.
            // Otherwise, return failure to the goal. (The timeout handler will send this failure.)
            ROS_WARN_STREAM("[" << param_node_name_ << "] : Resend mission ...");
            if (this->mission_status_ == mission_type::DOCK_TRACKER) {
                robot_dock_tracker_goal_pub_.publish(this->robot_goal_);
            } else {
                robot_path_tracker_goal_pub_.publish(this->robot_goal_);
            }
        }
    }
}

void Navigation_Main::MainMission_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    this->robot_goal_ = *msg;
    this->SetTimeout(this->robot_goal_.pose);
    this->start_time_ = ros::Time::now();
    this->is_mission_start_ = true;
    this->is_reach_goal_ = false;

    // Choose which mode we want to use here.
    if (msg->header.frame_id == "dock") {
        this->mission_status_ = mission_type::DOCK_TRACKER;
        robot_dock_tracker_goal_pub_.publish(this->robot_goal_);
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Dock mode.");
    } else {
        this->mission_status_ = mission_type::PATH_TRACKER;
        robot_path_tracker_goal_pub_.publish(this->robot_goal_);
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Path mode.");
    }

    ROS_INFO_STREAM("[" << param_node_name_ << "] : Mission start.");
}

void Navigation_Main::PathTrackerCmdVel_Callback(const geometry_msgs::Twist::ConstPtr &msgs) {
    if (this->mission_status_ == mission_type::PATH_TRACKER) {
        this->robot_cmd_vel_ = *msgs;
    }
}

void Navigation_Main::DockTrackerCmdVel_Callback(const geometry_msgs::Twist::ConstPtr &msgs) {
    if (this->mission_status_ == mission_type::DOCK_TRACKER) {
        this->robot_cmd_vel_ = *msgs;
    }
}

void Navigation_Main::DynamicParam_Callback(navigation_main::navigation_main_paramConfig &config, uint32_t level) {
    if (param_active_ != config.active) {
        this->param_active_ = config.active;
        ROS_INFO_STREAM("[" << param_node_name_ << "] : active set to " << param_active_);
    }
}