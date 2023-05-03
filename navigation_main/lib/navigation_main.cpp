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
        this->rival_obstacle_sub_.shutdown();
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
    robot_goal_.pose.position.x = robot_goal_.pose.position.y = robot_goal_.pose.orientation.z = -100.0;
    robot_odom_.position.x = robot_odom_.position.y = -100.0;
    robot_obs_odom_.position.x = robot_obs_odom_.position.y = -100.0;
    rival_odom_[0].position.x = rival_odom_[0].position.y = -100.0;
    rival_odom_[1].position.x = rival_odom_[1].position.y = -100.0;
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
    if (is_mission_start_ && isTimeout()) {
        FailToGoal();
    }

    Check_Odom_CB_Timeout();

    if ((mission_status_ == MISSION_TYPE::PATH_TRACKER || mission_status_ == MISSION_TYPE::DOCK_TRACKER)) {
        if (isCloseToOtherRobots()) {
            // The goal is too close to other robot.
            ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Too close to other robots)");
            HandleGoalUnreachable(false);
        } else if (isInBlockArea()) {
            // The goal is in the blocked area, surrounded by obstacles or robots.
            ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : In blocked area)");
            HandleGoalUnreachable(false);
        }
    } else if ((mission_status_ == MISSION_TYPE::STOP_PATH || mission_status_ == MISSION_TYPE::STOP_DOCK) && !isCloseToOtherRobots() && !isInBlockArea()) {
        HandleGoalUnreachable(true);
    }

    // ROS_INFO_STREAM("State : " << mission_status_);

    // Pub the robot cmd_vel to STM
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
    if (this->nh_local_->param<double>("block_mode_distance_a", param_block_mode_distance_a_, 1.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : block mode distance a set to " << param_block_mode_distance_a_);
    }
    if (this->nh_local_->param<double>("block_mode_distance_b", param_block_mode_distance_b_, 1.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : block mode distance b set to " << param_block_mode_distance_b_);
    }
    if (this->nh_local_->param<double>("block_mode_distance_c", param_block_mode_distance_c_, 1.0)) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : block mode distance c set to " << param_block_mode_distance_c_);
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
    if (this->nh_local_->param<std::string>("rival_obstacle_topic", param_rival_obstacle_topic_, "/RivalObstacle")) {
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Subscribe topic " << param_rival_obstacle_topic_);
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
            this->rival_obstacle_sub_ = this->nh_global_->subscribe(param_rival_obstacle_topic_, 100, &Navigation_Main::RivalObstacles_CB, this);
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
            this->resend_goal_timer_ = nh_local_->createTimer(ros::Duration(1.0 / param_resend_goal_frequency_), &Navigation_Main::ResendGoal_CB, this, false, false);

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
            this->rival_obstacle_sub_.shutdown();
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

void Navigation_Main::Check_Odom_CB_Timeout() {
    ros::Time current_time_ = ros::Time::now();

    is_robot_obs_odom_timeout_ = ((current_time_ - robot_obs_odom_time_).toSec() >= param_odom_timeout_) ? true : false;
    is_rival1_odom_timeout_ = ((current_time_ - rival_odom_time_[0]).toSec() >= param_odom_timeout_) ? true : false;
    is_rival2_odom_timeout_ = ((current_time_ - rival_odom_time_[1]).toSec() >= param_odom_timeout_) ? true : false;
    is_rival_obstacle_timeout_ = ((current_time_ - rival_obstacle_time_).toSec() >= param_odom_timeout_) ? true : false;
}

bool Navigation_Main::isCloseToOtherRobots() {
    if (!is_robot_obs_odom_timeout_ && Distance_Between_A_and_B(robot_goal_.pose, robot_obs_odom_) <= param_stop_distance_) {
        return true;
    }
    if (!is_rival1_odom_timeout_ && Distance_Between_A_and_B(robot_goal_.pose, rival_odom_[0]) <= param_stop_distance_) {
        return true;
    }
    if (!is_rival2_odom_timeout_ && Distance_Between_A_and_B(robot_goal_.pose, rival_odom_[1]) <= param_stop_distance_) {
        return true;
    }
    if (!is_rival_obstacle_timeout_) {
        geometry_msgs::Pose obstacle_pose_temp_;
        for (auto obstacle_ : rival_obstacles_.circles) {
            obstacle_pose_temp_.position.x = obstacle_.center.x;
            obstacle_pose_temp_.position.y = obstacle_.center.y;
            if (Distance_Between_A_and_B(robot_goal_.pose, obstacle_pose_temp_) <= param_stop_distance_) {
                return true;
            }
        }
    }

    return false;
}

double Navigation_Main::Distance_Between_A_and_B(const geometry_msgs::Pose poseA, const geometry_msgs::Pose poseB) {
    return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2));
}

double Navigation_Main::Distance_Between_A_and_B(const Point pointA, const Point pointB) {
    return sqrt(pow(pointA.x - pointB.x, 2) + pow(pointA.y - pointB.y, 2));
}

// Check whether goal is in the blocked area, which is defined by the closed area at corner.
bool Navigation_Main::isInBlockArea() {
    std::vector<Point> obstacle_point_;

    // Set the point that need to be consider
    if (!is_robot_obs_odom_timeout_) obstacle_point_.push_back({robot_obs_odom_.position.x, robot_obs_odom_.position.y});
    if (!is_rival1_odom_timeout_) obstacle_point_.push_back({rival_odom_[0].position.x, rival_odom_[0].position.y});
    if (!is_rival2_odom_timeout_) obstacle_point_.push_back({rival_odom_[1].position.x, rival_odom_[1].position.y});
    if (!is_rival_obstacle_timeout_) {
        for (auto obstacle_ : rival_obstacles_.circles) {
            obstacle_point_.push_back({obstacle_.center.x, obstacle_.center.y});
        }
    }

    if (obstacle_point_.empty()) return false;

    const int obstacle_point_size_ = obstacle_point_.size();
    static const double MAP_HALF_WIDTH = MAP_WIDTH / 2.0;
    static const double MAP_HALF_HEIGHT = MAP_HEIGHT / 2.0;

    // Choose every two points in obstacle_point to check.
    for (int i = 0; i < obstacle_point_size_; i++) {
        for (int j = i + 1; j < obstacle_point_size_; j++) {
            // Only calculate the points in same quadrant.
            if (((obstacle_point_[i].x - MAP_HALF_WIDTH) * (obstacle_point_[j].x - MAP_HALF_WIDTH) >= 0) &&
                ((obstacle_point_[i].y - MAP_HALF_HEIGHT) * (obstacle_point_[j].y - MAP_HALF_HEIGHT) >= 0)) {
                Point edge_point_[2];
                Point cherry_point;
                Point vertex_point;

                if (obstacle_point_[i].x <= MAP_HALF_WIDTH) {
                    edge_point_[0].x = std::max(obstacle_point_[i].x, obstacle_point_[j].x);
                    edge_point_[1] = {0, MAP_HALF_HEIGHT};
                    cherry_point = CHERRY_DISPENSER[0];

                    if (obstacle_point_[i].y <= MAP_HALF_HEIGHT) {
                        edge_point_[0].y = 0.0;
                        vertex_point = {0.0, 0.0};
                    } else {
                        edge_point_[0].y = MAP_HEIGHT;
                        vertex_point = {0.0, MAP_HEIGHT};
                    }
                } else {
                    edge_point_[0].x = std::min(obstacle_point_[i].x, obstacle_point_[j].x);
                    edge_point_[1] = {MAP_WIDTH, MAP_HALF_HEIGHT};
                    cherry_point = CHERRY_DISPENSER[1];

                    if (obstacle_point_[i].y <= MAP_HALF_HEIGHT) {
                        edge_point_[0].y = 0.0;
                        vertex_point = {MAP_WIDTH, 0.0};
                    } else {
                        edge_point_[0].y = MAP_HEIGHT;
                        vertex_point = {MAP_WIDTH, MAP_HEIGHT};
                    }
                }

                Point polygon_[6];
                if (edge_point_[0].x == obstacle_point_[i].x) {
                    polygon_[0] = obstacle_point_[i];
                    polygon_[1] = obstacle_point_[j];
                } else {
                    polygon_[0] = obstacle_point_[j];
                    polygon_[1] = obstacle_point_[i];
                }
                polygon_[2] = cherry_point;
                polygon_[3] = edge_point_[1];
                polygon_[4] = vertex_point;
                polygon_[5] = edge_point_[0];

                bool isGoalInPolygon = isPointInPolygon({robot_goal_.pose.position.x, robot_goal_.pose.position.y}, polygon_, 6);
                bool isRobotInPolygon = isPointInPolygon({robot_odom_.position.x, robot_odom_.position.y}, polygon_, 6);

                // Check whether both robot and goal in block area.
                if (isGoalInPolygon != isRobotInPolygon) {
                    // a -> obs1 to cherry
                    // b -> obs1 to obs2
                    // c -> obs2 to edge

                    if (Distance_Between_A_and_B(obstacle_point_[i], obstacle_point_[j]) <= param_block_mode_distance_b_) {
                        if (edge_point_[0].x == obstacle_point_[i].x) {
                            if (Distance_Between_A_and_B(cherry_point, obstacle_point_[j]) <= param_block_mode_distance_a_ &&
                                fabs(edge_point_[0].y - obstacle_point_[i].y) <= param_block_mode_distance_c_) {
                                return true;
                            }
                        } else {
                            if (Distance_Between_A_and_B(cherry_point, obstacle_point_[i]) <= param_block_mode_distance_a_ &&
                                fabs(edge_point_[0].y - obstacle_point_[j].y) <= param_block_mode_distance_c_) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}

// Reference: =.= ChatGPT =.=
bool Navigation_Main::isPointInPolygon(const Point point, const Point polygon[], int polygon_size) {
    double minX = polygon[0].x;
    double maxX = polygon[0].x;
    double minY = polygon[0].y;
    double maxY = polygon[0].y;

    for (int i = 1; i < polygon_size; i++) {
        minX = std::min(polygon[i].x, minX);
        maxX = std::max(polygon[i].x, maxX);
        minY = std::min(polygon[i].y, minY);
        maxY = std::max(polygon[i].y, maxY);
    }

    if (point.x < minX || point.x > maxX || point.y < minY || point.y > maxY)
        return false;

    bool is_in_polygon_ = false;

    for (int i = 0, j = polygon_size - 1; i < polygon_size; j = i++) {
        if ((polygon[i].y > point.y) != (polygon[j].y > point.y) && point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x) {
            is_in_polygon_ = !is_in_polygon_;
        }
    }

    return is_in_polygon_;
}

void Navigation_Main::HandleGoalUnreachable(bool reachable) {
    if (reachable) {
        // Restart to finish the mission.
        if (mission_status_ == MISSION_TYPE::STOP_DOCK) {
            mission_status_ = MISSION_TYPE::DOCK_TRACKER;
            robot_dock_tracker_goal_pub_.publish(this->robot_goal_);
        } else {
            mission_status_ = MISSION_TYPE::PATH_TRACKER;
            robot_path_tracker_goal_pub_.publish(this->robot_goal_);
        }

        resend_goal_timer_.stop();
    } else {
        // If there has some remaining time, retry to finish the mission.
        // Otherwise, return failure to the goal. (The timeout handler will send this failure.)
        robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0.0;
        geometry_msgs::PoseStamped interrupt_tracker_ = this->robot_goal_;
        interrupt_tracker_.pose.position.x = interrupt_tracker_.pose.position.y = interrupt_tracker_.pose.orientation.z = interrupt_tracker_.pose.orientation.w = -1.0;

        if (mission_status_ == MISSION_TYPE::DOCK_TRACKER) {
            mission_status_ = MISSION_TYPE::STOP_DOCK;
            robot_dock_tracker_goal_pub_.publish(interrupt_tracker_);
        } else {
            mission_status_ = MISSION_TYPE::STOP_PATH;
            robot_path_tracker_goal_pub_.publish(interrupt_tracker_);
        }

        // Start the timer to count up
        resend_goal_time_ = 0;
        resend_goal_timer_.start();
    }
}

// Callback function
void Navigation_Main::Robot_Odom_type0_CB(const nav_msgs::Odometry::ConstPtr &msg) {
    robot_odom_ = msg->pose.pose;
}
void Navigation_Main::Robot_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_odom_ = msg->pose.pose;
}

void Navigation_Main::RobotMissionState_CB(const std_msgs::Char::ConstPtr &msg) {
    if (is_mission_start_) {
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

// msg frame id format : (path/dock) _ (timeout for resend goal) _ (other info)
void Navigation_Main::MainMission_CB(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    this->robot_goal_ = *msg;
    this->start_time_ = ros::Time::now();
    this->is_mission_start_ = true;
    this->is_reach_goal_ = false;
    this->resend_goal_timer_.stop();

    // Choose which mode we want to use here.
    int timeout_idx_ = 5;
    int str_len_ = msg->header.frame_id.length();
    for (; timeout_idx_ < str_len_; timeout_idx_++) {
        if (msg->header.frame_id[timeout_idx_] == '_') {
            break;
        }
    }

    // Default timeout : 15 seconds.
    if (str_len_ == 4) {
        param_resend_goal_time_ = param_timeout_ = 15 * param_resend_goal_frequency_;
    } else {
        param_resend_goal_time_ = param_timeout_ = stoi(msg->header.frame_id.substr(5, timeout_idx_)) * param_resend_goal_frequency_;
    }

    // Set the msg that transmit to tracker.
    if (timeout_idx_ == str_len_ || str_len_ == 4) {
        this->robot_goal_.header.frame_id = msg->header.frame_id.substr(0, 4);
    } else {
        this->robot_goal_.header.frame_id = msg->header.frame_id.substr(0, 4) + msg->header.frame_id.substr(timeout_idx_, str_len_);
    }

    // Choose the tracker for transmit.
    if (robot_goal_.header.frame_id.substr(0, 4) == "dock") {
        this->mission_status_ = MISSION_TYPE::DOCK_TRACKER;
        robot_dock_tracker_goal_pub_.publish(this->robot_goal_);
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Dock mode : timeout " << param_resend_goal_time_ / param_resend_goal_frequency_);
    } else {
        this->mission_status_ = MISSION_TYPE::PATH_TRACKER;
        robot_path_tracker_goal_pub_.publish(this->robot_goal_);
        ROS_INFO_STREAM("[" << param_node_name_ << "] : Path mode : timeout " << param_resend_goal_time_ / param_resend_goal_frequency_);
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
    is_robot_obs_odom_timeout_ = false;
    robot_obs_odom_time_ = ros::Time::now();
}
void Navigation_Main::Robot_Obs_Odom_type1_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robot_obs_odom_ = msg->pose.pose;
    is_robot_obs_odom_timeout_ = false;
    robot_obs_odom_time_ = ros::Time::now();
}
void Navigation_Main::Rival1_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg) {
    rival_odom_[0] = msg->pose.pose;
    is_rival1_odom_timeout_ = false;
    rival_odom_time_[0] = ros::Time::now();
}
void Navigation_Main::Rival2_Odom_CB(const nav_msgs::Odometry::ConstPtr &msg) {
    rival_odom_[1] = msg->pose.pose;
    is_rival2_odom_timeout_ = false;
    rival_odom_time_[1] = ros::Time::now();
}
void Navigation_Main::RivalObstacles_CB(const obstacle_detector::Obstacles::ConstPtr &msg) {
    rival_obstacles_ = *msg;
    is_rival_obstacle_timeout_ = false;
    rival_obstacle_time_ = ros::Time::now();
}

void Navigation_Main::FailToGoal() {
    if (mission_status_ == MISSION_TYPE::PATH_TRACKER || mission_status_ == MISSION_TYPE::DOCK_TRACKER) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Timeout)");
    } else if (mission_status_ == MISSION_TYPE::RESEND_PATH_GOAL || mission_status_ == MISSION_TYPE::RESEND_DOCK_GOAL) {
        ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Timeout for resending goal)");
    } else if (mission_status_ == MISSION_TYPE::STOP_DOCK || mission_status_ == MISSION_TYPE::STOP_PATH) {
        if (isCloseToOtherRobots()) {
            ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : Too close to other robots)");
        } else {
            ROS_WARN_STREAM("[" << param_node_name_ << "] : Fail to the goal. (reason : In blocked area)");
        }
    }
    ROS_ERROR_STREAM("[" << param_node_name_ << "] : Mission Failed.");

    geometry_msgs::PoseStamped interrupt_tracker_ = this->robot_goal_;
    interrupt_tracker_.pose.position.x = interrupt_tracker_.pose.position.y = interrupt_tracker_.pose.orientation.z = interrupt_tracker_.pose.orientation.w = -1.0;
    robot_path_tracker_goal_pub_.publish(interrupt_tracker_);
    robot_dock_tracker_goal_pub_.publish(interrupt_tracker_);

    is_mission_start_ = false;

    // Publish to main for fail to finish mission.
    std_msgs::Bool temp;
    temp.data = false;
    main_mission_state_pub_.publish(temp);

    mission_status_ = MISSION_TYPE::IDLE;
    robot_cmd_vel_.linear.x = robot_cmd_vel_.linear.y = robot_cmd_vel_.angular.z = 0.0;
    resend_goal_timer_.stop();
}