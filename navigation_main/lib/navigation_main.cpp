#include "navigation_main/navigation_main.h"

Navigation_Main navigation_main_;

Navigation_Main::Navigation_Main() {
}

Navigation_Main::~Navigation_Main() {
    if (param_active_) {
        this->robot_odom_sub_.shutdown();
        this->robot_obs_odom_sub_.shutdown();
        this->rival_odom_sub_[0].shutdown();
        this->rival_odom_sub_[1].shutdown();
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
    nh_global_ = nh_global;
    nh_local_ = nh_local;
    param_node_name_ = node_name;
    robot_odom_.position.x = robot_odom_.position.y = 0.0;
    robot_obs_odom_.position.x = robot_obs_odom_.position.y = 0.0;
    rival_odom_[0].position.x = rival_odom_[0].position.y = 0.0;
    rival_odom_[1].position.x = rival_odom_[1].position.y = 0.0;
    robot_obs_odom_time_ = rival_odom_time_[0] = rival_odom_time_[1] = ros::Time::now();
    mission_status_ = MISSION_TYPE::IDLE;

    std_srvs::Empty empty;

    param_active_ = false;
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
    // Fail to reach the goal (Timeout)
    if (param_active_ && is_mission_start_ && isTimeout()) {
        FailToGoal();
    }

    if ((mission_status_ == MISSION_TYPE::PATH_TRACKER || mission_status_ == MISSION_TYPE::DOCK_TRACKER) && isCloseToOtherRobots()) {
        // The goal is too close to other robot.
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Too close to other robots)");

        // If there has some time remaining, retry to finish the mission.
        // Otherwise, return failure to the goal. (The timeout handler will send this failure.)
        robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0.0;
        if (mission_status_ == MISSION_TYPE::DOCK_TRACKER) {
            mission_status_ = MISSION_TYPE::STOP_DOCK;
        } else {
            mission_status_ = MISSION_TYPE::STOP_PATH;
        }

        // Start the resend timer;
        resend_goal_time_ = 0;
        resend_goal_timer_.start();
    } else if ((mission_status_ == MISSION_TYPE::STOP_PATH || mission_status_ == MISSION_TYPE::STOP_DOCK) && !isCloseToOtherRobots()) {
        if (mission_status_ == MISSION_TYPE::STOP_DOCK) {
            mission_status_ = MISSION_TYPE::DOCK_TRACKER;
        } else {
            mission_status_ = MISSION_TYPE::PATH_TRACKER;
        }
        resend_goal_timer_.stop();
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
        this->odom_type_ = (temp_odom_type_ == 0) ? ODOM_CALLBACK_TYPE::nav_msgs_Odometry : ODOM_CALLBACK_TYPE::geometry_msgs_PoseWithCovarianceStamped;
        ROS_INFO_STREAM("[" << param_node_name_ << "] : odom type set to " << temp_odom_type_);
    }
    if (this->nh_local_->param<int>("resend_goal_time", param_resend_goal_time_, 1)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : resend goal time set to " << param_resend_goal_time_);
    }
    if (this->nh_local_->param<double>("resend_goal_frequency", param_resend_goal_frequency_, 1.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : resend goal frequency set to " << param_resend_goal_frequency_);
    }
    if (this->nh_local_->param<double>("update_frequency", param_update_frequency_, 5.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : update frequency set to " << param_update_frequency_);
    }
    if (this->nh_local_->param<double>("stop_distance", param_stop_distance_, 0.5)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : stop distance set to " << param_stop_distance_);
    }
    if (this->nh_local_->param<double>("odom_timeout", param_odom_timeout_, 3.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : odom timeout set to " << param_odom_timeout_);
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
    if (this->nh_local_->param<std::string>("robot_odom_obs_topic", param_robot_obs_odom_topic_, "/robot1/odom")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_robot_obs_odom_topic_);
    }
    if (this->nh_local_->param<std::string>("rival1_odom_topic", param_rival_odom_topic_[0], "/rival1/odom")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_rival_odom_topic_[0]);
    }
    if (this->nh_local_->param<std::string>("rival2_odom_topic", param_rival_odom_topic_[1], "/rival2/odom")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_rival_odom_topic_[1]);
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
                case ODOM_CALLBACK_TYPE::nav_msgs_Odometry:
                    this->robot_odom_sub_ = this->nh_global_->subscribe(param_robot_odom_topic_, 100, &Navigation_Main::Robot_Odom_type0_CB, this);
                    this->robot_obs_odom_sub_ = this->nh_global_->subscribe(param_robot_obs_odom_topic_, 100, &Navigation_Main::Robot_Obs_Odom_type0_CB, this);
                    break;
                case ODOM_CALLBACK_TYPE::geometry_msgs_PoseWithCovarianceStamped:
                    this->robot_odom_sub_ = this->nh_global_->subscribe(param_robot_odom_topic_, 100, &Navigation_Main::Robot_Odom_type1_CB, this);
                    this->robot_obs_odom_sub_ = this->nh_global_->subscribe(param_robot_obs_odom_topic_, 100, &Navigation_Main::Robot_Obs_Odom_type1_CB, this);
                    break;
            }
            this->rival_odom_sub_[0] = this->nh_global_->subscribe(param_rival_odom_topic_[0], 100, &Navigation_Main::Rival1_Odom_CB, this);
            this->rival_odom_sub_[1] = this->nh_global_->subscribe(param_rival_odom_topic_[1], 100, &Navigation_Main::Rival2_Odom_CB, this);
            this->robot_mission_state_sub_ = this->nh_global_->subscribe(param_robot_mission_state_topic_, 100, &Navigation_Main::RobotMissionState_CB, this);
            this->main_mission_state_sub_ = this->nh_global_->subscribe(param_main_mission_topic_, 100, &Navigation_Main::MainMission_CB, this);
            this->robot_path_tracker_cmd_vel_sub_ = this->nh_global_->subscribe(param_robot_path_tracker_cmd_vel_topic_, 100, &Navigation_Main::PathTrackerCmdVel_CB, this);
            this->robot_dock_tracker_cmd_vel_sub_ = this->nh_global_->subscribe(param_robot_dock_tracker_cmd_vel_topic_, 100, &Navigation_Main::DockTrackerCmdVel_CB, this);

            // Publish
            this->robot_cmd_vel_pub_ = this->nh_global_->advertise<geometry_msgs::Twist>(param_robot_cmd_vel_topic_, 100);
            this->main_mission_state_pub_ = this->nh_global_->advertise<std_msgs::Bool>(param_main_mission_state_topic_, 100);
            this->robot_path_tracker_goal_pub_ = this->nh_global_->advertise<geometry_msgs::PoseStamped>(param_robot_path_tracker_goal_topic_, 100);
            this->robot_dock_tracker_goal_pub_ = this->nh_global_->advertise<geometry_msgs::PoseStamped>(param_robot_dock_tracker_goal_topic_, 100);

            // Create Timer for resend goal.
            this->resend_goal_timer_ = nh_local_->createTimer(ros::Duration(param_resend_goal_frequency_), &Navigation_Main::ResendGoal_CB, this, false, false);

            // Service for Update params.
            if (this->param_update_params_) {
                this->param_srv_ = nh_local_->advertiseService("params", &Navigation_Main::UpdateParams, this);
            }

            // Init Dynamic Reconfigure
            if (this->param_use_dynamic_reconfigure_) {
                this->SetDynamicReconfigure();
            }
        } else {
            this->robot_odom_sub_.shutdown();
            this->robot_obs_odom_sub_.shutdown();
            this->rival_odom_sub_[0].shutdown();
            this->rival_odom_sub_[1].shutdown();
            this->robot_mission_state_sub_.shutdown();
            this->robot_path_tracker_goal_pub_.shutdown();
            this->robot_dock_tracker_goal_pub_.shutdown();
            this->robot_cmd_vel_pub_.shutdown();
            this->robot_path_tracker_cmd_vel_sub_.shutdown();
            this->robot_dock_tracker_cmd_vel_sub_.shutdown();
            this->main_mission_state_sub_.shutdown();
            this->main_mission_state_pub_.shutdown();

            this->resend_goal_timer_.stop();

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

    callback = boost::bind(&Navigation_Main::DynamicParam_CB, this, _1, _2);

    // Set callback function to param server
    dynamic_param_srv_.setCallback(callback);
}

// Total Timeout = min_timeout < [(Distance to Goal) * (timeout_a) + (timeout_b)] < max_timeout
void Navigation_Main::SetTimeout(geometry_msgs::Pose poseGoal) {
    this->param_timeout_ = std::min(std::max(Distance_Between_A_and_B(poseGoal, robot_odom_) * param_timeout_a_ + param_timeout_b_, param_timeout_min_), param_timeout_max_);
    ROS_INFO_STREAM("[" << param_node_name_ << "] : set timeout to " << param_timeout_);
}

bool Navigation_Main::isCloseToOtherRobots() {
    return (Distance_Between_A_and_B(robot_goal_.pose, robot_obs_odom_) <= param_stop_distance_ || Distance_Between_A_and_B(robot_goal_.pose, rival_odom_[0]) <= param_stop_distance_ || Distance_Between_A_and_B(robot_goal_.pose, rival_odom_[1]) <= param_stop_distance_);
}

double Navigation_Main::Distance_Between_A_and_B(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB) {
    // ROS_INFO_STREAM("Distance : " << sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2)));
    return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2));
}

// Callback function
void Navigation_Main::Robot_Odom_type0_CB(const nav_msgs::Odometry::ConstPtr &msg) {
    robot_odom_ = msg->pose.pose;
}
void Navigation_Main::Robot_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_odom_ = msg->pose.pose;
}

void Navigation_Main::RobotMissionState_CB(const std_msgs::Char::ConstPtr &msg) {
    if (param_active_ && is_mission_start_) {
        if (msg->data == 1) {
            is_mission_start_ = false;
            is_reach_goal_ = true;

            // Publish to main for finish mission.
            std_msgs::Bool temp;
            temp.data = true;
            main_mission_state_pub_.publish(temp);

            this->mission_status_ = MISSION_TYPE::IDLE;
            robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0.0;
            resend_goal_timer_.stop();
            ROS_INFO_STREAM("[" << param_node_name_ << "] : Mission end.");

        } else if (msg->data == 2) {
            // The goal is blocked.
            is_reach_goal_ = false;
            if (mission_status_ != MISSION_TYPE::RESEND_PATH_GOAL && mission_status_ != MISSION_TYPE::RESEND_DOCK_GOAL) {
                ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Unreachable Goal)");

                // If there has some time remaining, retry to finish the mission.
                // Otherwise, return failure to the goal. (The timeout handler will send this failure.)
                robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0.0;
                if (mission_status_ == MISSION_TYPE::DOCK_TRACKER) {
                    mission_status_ = MISSION_TYPE::RESEND_DOCK_GOAL;
                } else {
                    mission_status_ = MISSION_TYPE::RESEND_PATH_GOAL;
                }

                // Start the resend timer;
                resend_goal_time_ = 0;
                resend_goal_timer_.start();
            }
        } else if (msg->data == 0 && (mission_status_ == MISSION_TYPE::RESEND_PATH_GOAL || mission_status_ == MISSION_TYPE::RESEND_DOCK_GOAL)) {
            // The goal is not blocked, going to goal.
            is_reach_goal_ = false;
            resend_goal_timer_.stop();
            if (mission_status_ == MISSION_TYPE::RESEND_PATH_GOAL) {
                mission_status_ = MISSION_TYPE::PATH_TRACKER;
            } else if (mission_status_ == MISSION_TYPE::RESEND_DOCK_GOAL) {
                mission_status_ = MISSION_TYPE::DOCK_TRACKER;
            }
        }
    }
}

void Navigation_Main::MainMission_CB(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    this->robot_goal_ = *msg;
    this->SetTimeout(this->robot_goal_.pose);
    this->start_time_ = ros::Time::now();
    this->is_mission_start_ = true;
    this->is_reach_goal_ = false;
    this->resend_goal_timer_.stop();

    // Choose which mode we want to use here.
    if (msg->header.frame_id.substr(0, 4) == "dock") {
        this->mission_status_ = MISSION_TYPE::DOCK_TRACKER;
        robot_dock_tracker_goal_pub_.publish(this->robot_goal_);
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Dock mode.");
    } else {
        this->mission_status_ = MISSION_TYPE::PATH_TRACKER;
        robot_path_tracker_goal_pub_.publish(this->robot_goal_);
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Path mode.");
    }

    ROS_INFO_STREAM("[" << param_node_name_ << "] : Mission start.");
}

void Navigation_Main::PathTrackerCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msgs) {
    if (mission_status_ == MISSION_TYPE::PATH_TRACKER) {
        robot_cmd_vel_ = *msgs;
    }
}

void Navigation_Main::DockTrackerCmdVel_CB(const geometry_msgs::Twist::ConstPtr &msgs) {
    if (mission_status_ == MISSION_TYPE::DOCK_TRACKER) {
        robot_cmd_vel_ = *msgs;
    }
}

void Navigation_Main::DynamicParam_CB(const navigation_main::navigation_main_paramConfig &config, uint32_t level) {
    if (param_active_ != config.active) {
        this->param_active_ = config.active;
        ROS_INFO_STREAM("[" << param_node_name_ << "] : active set to " << param_active_);
    }
}

void Navigation_Main::ResendGoal_CB(const ros::TimerEvent &event) {
    if (++resend_goal_time_ > param_resend_goal_time_) {
        FailToGoal();
        return;
    }

    if (mission_status_ == MISSION_TYPE::RESEND_DOCK_GOAL) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Resend mission : " << resend_goal_time_ << "th times.");
        robot_dock_tracker_goal_pub_.publish(robot_goal_);
    } else if (mission_status_ == MISSION_TYPE::RESEND_PATH_GOAL) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Resend mission : " << resend_goal_time_ << "th times.");
        robot_path_tracker_goal_pub_.publish(robot_goal_);
    } else if (mission_status_ == MISSION_TYPE::STOP_PATH || mission_status_ == MISSION_TYPE::STOP_DOCK) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Wait Robot leave : " << resend_goal_time_ << "th times.");
    }
}

void Navigation_Main::Robot_Obs_Odom_type0_CB(const nav_msgs::Odometry::ConstPtr &msg) {
    robot_obs_odom_ = msg->pose.pose;
    robot_obs_odom_time_ = ros::Time::now();
}
void Navigation_Main::Robot_Obs_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_obs_odom_ = msg->pose.pose;
    robot_obs_odom_time_ = ros::Time::now();
}
void Navigation_Main::Rival1_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg) {
    rival_odom_[0] = msg->pose.pose;
    rival_odom_time_[0] = ros::Time::now();
}
void Navigation_Main::Rival2_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg) {
    rival_odom_[1] = msg->pose.pose;
    rival_odom_time_[1] = ros::Time::now();
}

void Navigation_Main::FailToGoal() {
    if (mission_status_ == MISSION_TYPE::PATH_TRACKER || mission_status_ == MISSION_TYPE::DOCK_TRACKER) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Timeout)");
    } else if (mission_status_ == MISSION_TYPE::RESEND_PATH_GOAL || mission_status_ == MISSION_TYPE::RESEND_DOCK_GOAL) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Timeout for resending goal)");
    } else if (mission_status_ == MISSION_TYPE::STOP_DOCK || mission_status_ == MISSION_TYPE::STOP_PATH) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Too close to other robots)");
    }
    ROS_ERROR_STREAM("[" << param_node_name_ << "] : Mission Failed.");

    is_mission_start_ = false;

    // Publish to main for fail to finish mission.
    std_msgs::Bool temp;
    temp.data = false;
    main_mission_state_pub_.publish(temp);

    this->mission_status_ = MISSION_TYPE::IDLE;
    this->robot_cmd_vel_.linear.x = this->robot_cmd_vel_.linear.y = this->robot_cmd_vel_.angular.z = 0.0;
    resend_goal_timer_.stop();
}