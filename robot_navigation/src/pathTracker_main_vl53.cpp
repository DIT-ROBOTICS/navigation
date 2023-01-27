#include "pathTracker_main_vl53.h"
#include <cmath>

using namespace std;

RobotState::RobotState(double x, double y, double theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos)
{
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}

Eigen::Vector3d RobotState::getVector()
{
    Eigen::Vector3d vec;
    vec << x_, y_, theta_;
    return vec;
}

pathTracker::pathTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &pathTracker::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

pathTracker::~pathTracker()
{
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("lookahead_distance");

    nh_local_.deleteParam("linear_kp");
    nh_local_.deleteParam("linear_max_velocity");
    nh_local_.deleteParam("linear_acceleration");
    nh_local_.deleteParam("linear_brake_distance");
    nh_local_.deleteParam("linear_brake_distance_ratio");
    nh_local_.deleteParam("linear_min_brake_distance");
    nh_local_.deleteParam("xy_tolerance");
    nh_local_.deleteParam("linear_transition_vel_");
    nh_local_.deleteParam("linear_transition_acc_");
    nh_local_.deleteParam("linear_acceleration_profile");
    nh_local_.deleteParam("linear_deceleration_profile");

    nh_local_.deleteParam("angular_kp");
    nh_local_.deleteParam("angular_max_velocity");
    nh_local_.deleteParam("angular_acceleration");
    nh_local_.deleteParam("angular_brake_distance");
    nh_local_.deleteParam("theta_tolerance");
    nh_local_.deleteParam("angular_transition_vel_");
    nh_local_.deleteParam("angular_transition_acc_");
    nh_local_.deleteParam("angular_acceleration_profile");
    nh_local_.deleteParam("angular_deceleration_profile");

    nh_local_.deleteParam("vl53_r_dis");
    nh_local_.deleteParam("vl53_g_dis");
    nh_local_.deleteParam("vl53_b_dis");
    nh_local_.deleteParam("docking_xy_tolerance");
    nh_local_.deleteParam("docking_theta_tolerance");
    nh_local_.deleteParam("docking_theta_p");
    nh_local_.deleteParam("docking_linear_kp");
    nh_local_.deleteParam("docking_angular_kp");
    nh_local_.deleteParam("max_docking_theta");
    nh_local_.deleteParam("one_eye_docking_min");
    nh_local_.deleteParam("docking_timeout");
}

void pathTracker::initialize()
{
    if_localgoal_final_reached = false;
    if_globalpath_switched = false;
    if_obstacle_surr = false;
    max_linear_vel_reached_ = 0.0;
    r_arm_angle = 90 * M_PI / 180;
    g_arm_angle = 210 * M_PI / 180;
    b_arm_angle = 330 * M_PI / 180;
    goal_flag_ = false;
    docking_flag_ = false;
    one_eye_docking = true;

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &pathTracker::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();
    workingMode_ = Mode::IDLE;
    workingMode_past_ = Mode::IDLE;

    ROS_INFO_THROTTLE(1, "Initialized !");
}

bool pathTracker::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    // load parameter
    bool get_param_ok = true;
    bool prev_active = p_active_;

    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    get_param_ok = nh_local_.param<string>("robot_type", robot_type_, "omni");
    get_param_ok = nh_local_.param<double>("control_frequency", control_frequency_, 50);
    get_param_ok = nh_local_.param<double>("lookahead_distance", lookahead_d_, 0.2);

    // linear parameter
    // acceleration
    get_param_ok = nh_local_.param<double>("linear_max_velocity", linear_max_vel_, 0.5);
    get_param_ok = nh_local_.param<double>("linear_acceleration", linear_acceleration_, 0.3);
    get_param_ok = nh_local_.param<string>("linear_acceleration_profile", linear_acceleration_profile_, "linear");
    // transition
    get_param_ok = nh_local_.param<double>("linear_transition_velocity", linear_transition_vel_, 0.15);
    get_param_ok = nh_local_.param<double>("linear_transition_acceleration", linear_transition_acc_, 0.6);
    // deceleration
    get_param_ok = nh_local_.param<double>("linear_kp", linear_kp_, 0.8);
    get_param_ok = nh_local_.param<double>("linear_brake_distance_ratio", linear_brake_distance_ratio_, 0.3);
    get_param_ok = nh_local_.param<double>("linear_min_brake_distance", linear_min_brake_distance_, 0.3);
    get_param_ok = nh_local_.param<string>("linear_deceleration_profile", linear_deceleration_profile_, "linear");

    // angular parameter
    get_param_ok = nh_local_.param<double>("angular_max_velocity", angular_max_vel_, 3);
    get_param_ok = nh_local_.param<double>("angular_acceleration", angular_acceleration_, 0.5);
    get_param_ok = nh_local_.param<double>("angular_brake_distance", angular_brake_distance_, 0.35);
    get_param_ok = nh_local_.param<double>("angular_transition_velocity", angular_transition_vel_, 0.15);
    get_param_ok = nh_local_.param<double>("angular_transition_acceleration", angular_transition_acc_, 0.6);
    get_param_ok = nh_local_.param<double>("angular_kp", angular_kp_, 1.5);
    get_param_ok = nh_local_.param<string>("angular_acceleration_profile", angular_acceleration_profile_, "linear");
    get_param_ok = nh_local_.param<string>("angular_deceleration_profile", angular_deceleration_profile_, "linear");

    get_param_ok = nh_local_.param<double>("xy_tolerance", xy_tolerance_, 0.01);
    get_param_ok = nh_local_.param<double>("theta_tolerance", theta_tolerance_, 0.03);

    // docking
    get_param_ok = nh_local_.param<double>("vl53_r_dis", vl53_r_dis_, 0.01);
    get_param_ok = nh_local_.param<double>("vl53_g_dis", vl53_g_dis_, 0.01);
    get_param_ok = nh_local_.param<double>("vl53_b_dis", vl53_b_dis_, 0.01);
    get_param_ok = nh_local_.param<double>("docking_xy_tolerance", docking_xy_tolerance_, 0.01);
    get_param_ok = nh_local_.param<double>("docking_theta_tolerance", docking_theta_tolerance_, 0.01);
    get_param_ok = nh_local_.param<double>("docking_theta_p", docking_theta_p_, 0.01);
    get_param_ok = nh_local_.param<double>("docking_const_linear", docking_const_linear_, 0.01);
    get_param_ok = nh_local_.param<double>("docking_const_angular", docking_const_angular_, 0.01);
    get_param_ok = nh_local_.param<double>("docking_linear_kp", docking_linear_kp_, 0.5);
    get_param_ok = nh_local_.param<double>("docking_angular_kp", docking_angular_kp_, 0.5);
    get_param_ok = nh_local_.param<double>("max_docking_theta", max_docking_theta_, 0.26);
    get_param_ok = nh_local_.param<double>("one_eye_docking_min", one_eye_docking_min_, 0.26);
    get_param_ok = nh_local_.param<double>("docking_timeout", docking_timeout_, 4.0);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            poseSub_ = nh_.subscribe("ekf_pose", 50, &pathTracker::poseCallback, this);
            // poseSub_ = nh_.subscribe("/global_filter", 50, &pathTracker::poseCallback, this);
            goalSub_ = nh_.subscribe("target", 50, &pathTracker::goalCallback, this);
            actionSub_ = nh_.subscribe("Stopornot", 10, &pathTracker::actionCallback, this);
            dockingSub_ = nh_.subscribe("tof_data", 10, &pathTracker::dockingCallback, this);
            dockingGoalSub_ = nh_.subscribe("docking_goal", 10, &pathTracker::dockingGoalCallback, this);

            velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
            localgoalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 10);
            posearrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("orientation", 10);
            goalreachedPub_ = nh_.advertise<std_msgs::Bool>("Finishornot", 1);
        }
        else
        {
            poseSub_.shutdown();
            goalSub_.shutdown();
            actionSub_.shutdown();
            dockingSub_.shutdown();
            velPub_.shutdown();
            localgoalPub_.shutdown();
            posearrayPub_.shutdown();
            goalreachedPub_.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Path Tracker]: "
                        << "set param ok");
    }
    else
    {
        ROS_WARN_STREAM("[Path Tracker]: "
                        << "set param failed");
    }
    cout << "param updated !" << endl;
    return true;
}

void pathTracker::dockingCallback(const std_msgs::Float64MultiArray::ConstPtr& docking_msg)
{
    docking_raw_data.clear();
    for (int i = 0; i < 6; i++)
    {
        docking_raw_data.push_back(docking_msg->data[i]);
    }
}

void pathTracker::dockingGoalCallback(const std_msgs::Float32MultiArray::ConstPtr& docking_msg)
{
    which_arm = docking_msg->data[0];
    goal_dis_left = docking_msg->data[1];
    goal_dis_right = docking_msg->data[2];
    docking_x_offset = docking_msg->data[3];
    docking_y_offset = docking_msg->data[4];
    docking_linear = false;
    docking_angular = false;
    total_docking_theta = 0;

    docking_flag_ = true;
    if (goal_flag_ && docking_flag_)
    {
        if (which_arm != 1)
        {
            goal_pose_.x_ += docking_x_offset;
            goal_pose_.y_ += docking_y_offset;
        }
        goal_flag_ = false;
        docking_flag_ = false;
        switchMode(Mode::GLOBALPATH_RECEIVED);
    }
}

void pathTracker::timerCallback(const ros::TimerEvent& e)
{
    if (if_obstacle_surr == true)
    {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
        velocity_state_.theta_ = 0;
        velocityPublish();
    }
    else
    {
        switch (workingMode_)
        {
            case Mode::GLOBALPATH_RECEIVED: {
                if (workingMode_past_ == Mode::IDLE)
                {
                    switchMode(Mode::TRACKING);
                    break;
                }
                else if (workingMode_past_ == Mode::TRACKING)
                {
                    // Slow down first then start tracking new path
                    switchMode(Mode::TRANSITION);
                    break;
                }
                else if (workingMode_past_ == Mode::TRANSITION)
                {
                    switchMode(Mode::TRACKING);
                    break;
                }
                else if (workingMode_past_ == Mode::DOCKING)
                {
                    switchMode(Mode::TRANSITION);
                    break;
                }
            }
            break;

            case Mode::TRACKING: {
                if (xy_goal_reached(cur_pose_, goal_pose_) && theta_goal_reached(cur_pose_, goal_pose_))
                {
                    if (which_arm == -1)
                    {
                        ROS_INFO("Working Mode : TRACKING GOAL REACHED !");
                        switchMode(Mode::IDLE);
                        velocity_state_.x_ = 0;
                        velocity_state_.y_ = 0;
                        velocity_state_.theta_ = 0;
                        velocityPublish();
                        std_msgs::Bool goalreached;
                        goalreached.data = true;
                        goalreachedPub_.publish(goalreached);
                        which_arm = -1;
                        docking_x_offset = 0;
                        docking_y_offset = 0;
                        break;
                    }
                    else
                    {
                        switchMode(Mode::DOCKING);
                        docking_timeout_t1 = ros::Time::now().toSec();
                    }
                }

                if (workingMode_past_ == Mode::TRANSITION)
                {
                    if (if_globalpath_switched == false)
                    {
                        if_localgoal_final_reached = false;
                        plannerClient(cur_pose_, goal_pose_);
                        linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
                        max_linear_vel_reached_ = 0;
                        if_globalpath_switched = true;
                    }
                }
                ROS_INFO("Working Mode : TRACKING");
                if (robot_type_ == "omni")
                {
                    RobotState local_goal;
                    local_goal = rollingWindow(cur_pose_, global_path_, lookahead_d_);
                    omniController(local_goal, cur_pose_);
                }
                else if (robot_type_ == "diff")
                {
                    RobotState local_goal;
                    local_goal = rollingWindow(cur_pose_, global_path_, lookahead_d_);
                    diffController(local_goal, cur_pose_);
                }
            }
            break;

            case Mode::IDLE: {
                // ROS_INFO_THROTTLE(1,"Working Mode : IDLE");
                velocity_state_.x_ = 0;
                velocity_state_.y_ = 0;
                velocity_state_.theta_ = 0;
                velocityPublish();
            }
            break;

            case Mode::TRANSITION: {
                // ROS_INFO_THROTTLE(1,"Working Mode : TRANSITION");
                double linear_vel = sqrt(pow(velocity_state_.x_, 2) + pow(velocity_state_.y_, 2));
                double angular_vel = velocity_state_.theta_;

                if (linear_vel <= linear_transition_vel_ && angular_vel <= angular_transition_vel_)
                {
                    switchMode(Mode::TRACKING);
                    break;
                }

                if (robot_type_ == "omni")
                {
                    RobotState local_goal;
                    local_goal = rollingWindow(cur_pose_, global_path_past_, lookahead_d_);
                    omniController(local_goal, cur_pose_);
                }
                else if (robot_type_ == "diff")
                {
                    RobotState local_goal;
                    local_goal = rollingWindow(cur_pose_, global_path_past_, lookahead_d_);
                    diffController(local_goal, cur_pose_);
                }
            }
            break;

            case Mode::DOCKING: {
                docking_timeout_t2 = ros::Time::now().toSec();
                double now_dis_right, now_dis_left, vl53_dis, vl53_angle;  // from docking callback

                switch (which_arm)
                {
                    case 82:  // R
                        now_dis_left = docking_raw_data[5];
                        now_dis_right = docking_raw_data[4];
                        vl53_dis = vl53_r_dis_;
                        vl53_angle = r_arm_angle;
                        break;
                    case 83:  // R
                        now_dis_left = docking_raw_data[5];
                        now_dis_right = docking_raw_data[4];
                        vl53_dis = vl53_r_dis_;
                        vl53_angle = r_arm_angle;
                        break;
                    case 71:  // G
                        now_dis_left = docking_raw_data[3];
                        now_dis_right = docking_raw_data[2];
                        vl53_dis = vl53_g_dis_;
                        vl53_angle = g_arm_angle;
                        break;
                    case 72:  // G
                        now_dis_left = docking_raw_data[3];
                        now_dis_right = docking_raw_data[2];
                        vl53_dis = vl53_g_dis_;
                        vl53_angle = g_arm_angle;
                        break;
                    case 66:  // B
                        now_dis_left = docking_raw_data[1];
                        now_dis_right = docking_raw_data[0];
                        vl53_dis = vl53_b_dis_;
                        vl53_angle = b_arm_angle;
                        break;
                    case 67:  // B
                        now_dis_left = docking_raw_data[1];
                        now_dis_right = docking_raw_data[0];
                        vl53_dis = vl53_b_dis_;
                        vl53_angle = b_arm_angle;
                        break;
                }

                double left_err = goal_dis_left - now_dis_left;
                double right_err = goal_dis_right - now_dis_right;
                double xy_err = (left_err + right_err) / 2.0;

                // ROS_INFO_THROTTLE(1, "total = %f /  max = %f", total_docking_theta, max_docking_theta_);
                if (now_dis_left == -1 || now_dis_right == -1 || fabs(total_docking_theta) > max_docking_theta_ ||
                    one_eye_docking == false)
                {
                    velocity_state_.x_ = 0;
                    velocity_state_.y_ = 0;
                    velocity_state_.theta_ = 0;
                    velocityPublish();
                    which_arm = -1;
                    ROS_INFO("-------------------");
                    ROS_INFO(" offset goal = %f  /  %f ", goal_pose_.x_, goal_pose_.y_);
                    goal_pose_.x_ -= docking_x_offset;
                    goal_pose_.y_ -= docking_y_offset;
                    ROS_INFO(" offset back goal = %f  /  %f ", goal_pose_.x_, goal_pose_.y_);
                    ROS_INFO("-------------------");
                    switchMode(Mode::TRACKING);
                    total_docking_theta = 0;
                    break;
                }

                ROS_INFO("goal_dis_left, now_dis_left = %f   %f", goal_dis_left, now_dis_left);
                ROS_INFO("goal_dis_right, now_dis_right = %f   %f", goal_dis_right, now_dis_right);

                // ROS_INFO("total_docking_theta = %f", total_docking_theta);
                if ((fabs(left_err) <= docking_xy_tolerance_ && fabs(right_err) <= docking_xy_tolerance_) ||
                    (docking_linear == true && docking_angular == true) ||
                    docking_timeout_t2 - docking_timeout_t1 > docking_timeout_)
                {
                    if (which_arm == 83 || which_arm == 72 || which_arm == 67)
                    {
                        docking_angular = true;
                        if (which_side == 1)  // yellow
                        {
                            right_err = left_err;
                        }
                        else if (which_side == 2)  // purple
                        {
                            left_err = right_err;
                        }

                        double actual_goal_x = goal_pose_.x_ - docking_x_offset;
                        if (fabs(cur_pose_.x_ - actual_goal_x) > one_eye_docking_min_)
                        {
                            // one_eye_docking = false;
                            one_eye_docking = true;
                            break;
                        }
                    }
                    ROS_INFO("Working Mode : DOCKING GOAL REACHED !");
                    switchMode(Mode::IDLE);
                    velocity_state_.x_ = 0;
                    velocity_state_.y_ = 0;
                    velocity_state_.theta_ = 0;
                    velocityPublish();
                    std_msgs::Bool goalreached;
                    goalreached.data = true;
                    goalreachedPub_.publish(goalreached);
                    which_arm = -1;
                    one_eye_docking = true;
                    break;
                }
                else
                {
                    // ROS_INFO_THROTTLE(1,"DOCKING!!!");

                    /*linear*/
                    // ROS_INFO_THROTTLE(1, "%f              %d              %d", xy_err, docking_linear,
                    // docking_angular);
                    if (fabs(xy_err) > docking_xy_tolerance_ && (docking_linear == false && docking_angular == true))
                    {
                        ROS_INFO_THROTTLE(1, "DOCKING!!! - - LLL");

                        // double xy_err = (left_err > right_err) ? (right_err : left_err);  // choose small

                        // velocity_state_.x_ = xy_err * cos(vl53_angle) * linear_kp_;
                        // velocity_state_.y_ = xy_err * sin(vl53_angle) * linear_kp_;
                        ROS_INFO_THROTTLE(1, "%f    %f    %f", left_err, right_err, xy_err);

                        double output_vel = fabs(xy_err * docking_linear_kp_);
                        if (output_vel > docking_const_linear_)
                        {
                            output_vel = docking_const_linear_;
                        }

                        if (xy_err > 0)
                        {
                            ROS_INFO("back");
                            velocity_state_.x_ = -1 * output_vel * cos(vl53_angle);
                            velocity_state_.y_ = -1 * output_vel * sin(vl53_angle);
                        }
                        else
                        {
                            ROS_INFO("forward");
                            velocity_state_.x_ = output_vel * cos(vl53_angle);
                            velocity_state_.y_ = output_vel * sin(vl53_angle);
                        }
                    }
                    else if (fabs(xy_err) <= docking_xy_tolerance_)
                    {
                        if (docking_angular == true)
                            docking_linear = true;
                        velocity_state_.x_ = 0.0;
                        velocity_state_.y_ = 0.0;
                    }
                    else
                    {
                        velocity_state_.x_ = 0.0;
                        velocity_state_.y_ = 0.0;
                    }

                    /*angular*/
                    // double goal_theta = atan(fabs(goal_dis_right - goal_dis_left) / vl53_dis);
                    // double now_theta = atan(fabs(now_dis_right - now_dis_left) / vl53_dis);
                    double goal_theta = atan((goal_dis_right - goal_dis_left) / vl53_dis);
                    double now_theta = atan((now_dis_right - now_dis_left) / vl53_dis);
                    double theta_err = angleLimitChecking(goal_theta - now_theta);

                    if (fabs(theta_err) > docking_theta_tolerance_ && docking_angular == false)
                    {
                        ROS_INFO_THROTTLE(1, "DOCKING!!! - - AAA");
                        ROS_INFO_THROTTLE(1, "goal_theta = %f", goal_theta);
                        ROS_INFO_THROTTLE(1, "now_theta = %f", now_theta);
                        ROS_INFO_THROTTLE(1, "theta_err = %f", theta_err);

                        double output_vel;
                        output_vel = fabs(theta_err) * docking_angular_kp_;
                        if (output_vel > docking_const_angular_)
                        {
                            output_vel = docking_const_angular_;
                        }

                        if (theta_err < 0)
                        {
                            output_vel = -1 * output_vel;
                        }
                        else
                        {
                            output_vel = output_vel;
                        }
                        velocity_state_.theta_ = output_vel;
                    }
                    else if (fabs(theta_err) <= docking_theta_tolerance_)
                    {
                        docking_angular = true;
                        velocity_state_.theta_ = 0.0;
                    }
                    else
                    {
                        velocity_state_.theta_ = 0.0;
                    }
                }
                total_docking_theta += velocity_state_.theta_ * (e.current_real.toSec() - e.last_real.toSec());
                velocityPublish();
            }
            break;
        }
    }
}

void pathTracker::switchMode(Mode next_mode)
{
    workingMode_past_ = workingMode_;
    workingMode_ = next_mode;
}

void pathTracker::plannerClient(RobotState cur_pos, RobotState goal_pos)
{
    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = "map";
    cur.pose.position.x = cur_pos.x_;
    cur.pose.position.y = cur_pos.y_;
    cur.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, cur_pos.theta_);
    cur.pose.orientation.x = q.x();
    cur.pose.orientation.y = q.y();
    cur.pose.orientation.z = q.z();
    cur.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_pos.x_;
    goal.pose.position.y = goal_pos.y_;
    goal.pose.position.z = 0;

    // tf2::Quaternion q;
    q.setRPY(0, 0, goal_pos.theta_);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("MissionPath");
    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    std::vector<geometry_msgs::PoseStamped> path_msg;

    if (client.call(srv))
    {
        ROS_INFO_THROTTLE(1, "Path received from global planner !");
        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;
        global_path_.clear();

        for (const auto& point : path_msg.poses)
        {
            RobotState pose;
            pose.x_ = point.pose.position.x;
            pose.y_ = point.pose.position.y;
            tf2::Quaternion q;
            tf2::fromMsg(point.pose.orientation, q);
            tf2::Matrix3x3 qt(q);
            double _, yaw;
            qt.getRPY(_, _, yaw);
            pose.theta_ = yaw;
            global_path_.push_back(pose);
        }
        global_path_ = orientationFilter(global_path_);
        ROS_INFO_THROTTLE(1, "Path received from global planner !");

        // print global path
        // ROS_INFO_THROTTLE(1,"--- global path ---");
        // for (const auto& point : global_path_)
        // {
        //     ROS_INFO_THROTTLE(1,"(%f, %f, %f)", point.x_, point.y_, point.theta_);
        // }
        // ROS_INFO_THROTTLE(1,"--- ---");
    }
    else
    {
        ROS_ERROR("Failed to call service make_plan");
        // return 1;
    }
}

void pathTracker::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    cur_pose_.x_ = pose_msg->pose.pose.position.x;
    cur_pose_.y_ = pose_msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose_.theta_ = yaw;
}

// void pathTracker::poseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
// {
//     cur_pose_.x_ = pose_msg->pose.pose.position.x;
//     cur_pose_.y_ = pose_msg->pose.pose.position.y;
//     tf2::Quaternion q;
//     tf2::fromMsg(pose_msg->pose.pose.orientation, q);
//     tf2::Matrix3x3 qt(q)
//     double _, yaw;
//     qt.getRPY(_, _, yaw);
//     cur_pose_.theta_ = yaw;
// }

void pathTracker::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    nh_.getParam("side_state", which_side);
    ROS_INFO("which_side = %d", which_side);

    std_srvs::Empty empt;
    initializeParams(empt.request, empt.response);

    goal_pose_.x_ = pose_msg->pose.position.x;
    goal_pose_.y_ = pose_msg->pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    goal_pose_.theta_ = yaw;

    ROS_INFO_THROTTLE(1, "Goal received ! (%f, %f, %f)", goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);

    global_path_past_ = global_path_;

    if (workingMode_ == Mode::IDLE)
    {
        plannerClient(cur_pose_, goal_pose_);
        linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
        max_linear_vel_reached_ = 0;
        // if (linear_brake_distance_ < linear_min_brake_distance_)
        //     linear_brake_distance_ = linear_min_brake_distance_;
        if_globalpath_switched = true;
    }
    else
    {
        if_globalpath_switched = false;
    }

    if_localgoal_final_reached = false;

    goal_flag_ = true;
    if (goal_flag_ && docking_flag_)
    {
        if (which_arm != 1)
        {
            goal_pose_.x_ += docking_x_offset;
            goal_pose_.y_ += docking_y_offset;
        }
        goal_flag_ = false;
        docking_flag_ = false;
        switchMode(Mode::GLOBALPATH_RECEIVED);
    }
}

void pathTracker::actionCallback(const std_msgs::Bool::ConstPtr& action_msg)
{
    if_obstacle_surr = action_msg->data;
}

RobotState pathTracker::rollingWindow(RobotState cur_pos, std::vector<RobotState> path, double L_d)
{
    int k = 1;
    int last_k = 0;
    int d_k = 0;
    RobotState a;
    int a_idx = 0;
    RobotState b;
    int b_idx = 0;
    RobotState local_goal;
    bool if_b_asigned = false;
    double r = L_d;

    for (int i = 0; i < path.size(); i++)
    {
        if (i == 1)
            last_k = 0;
        last_k = k;
        if (cur_pos.distanceTo(path.at(i)) >= r)
            k = 1;
        else
            k = 0;

        d_k = k - last_k;

        if (d_k == 1)
        {
            b = path.at(i);
            if_b_asigned = true;
            b_idx = i;
            a_idx = i - 1;
            break;
        }
    }

    if (!if_b_asigned)
    {
        double min = 1000000;
        for (int i = 0; i < path.size(); i++)
        {
            if (cur_pos.distanceTo(path.at(i)) < min)
            {
                min = cur_pos.distanceTo(path.at(i));
                b_idx = i;
                a_idx = i - 1;
                b = path.at(i);
            }
        }
    }

    if (a_idx == -1)
    {
        local_goal = path.at(b_idx);
    }
    else
    {
        a = path.at(a_idx);
        double d_ca = cur_pos.distanceTo(a);
        double d_cb = cur_pos.distanceTo(b);
        local_goal.x_ = a.x_ + (b.x_ - a.x_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.y_ = a.y_ + (b.y_ - a.y_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.theta_ = a.theta_;
    }

    if (if_localgoal_final_reached)
    {
        // cout << "local goal set to path.back()" << endl;
        // local_goal = path.back();
        local_goal = goal_pose_;
    }

    if (cur_pos.distanceTo(path.back()) < r + 0.01)
        // local_goal = path.back();
        local_goal = goal_pose_;

    if (local_goal.distanceTo(path.back()) < 0.05)
    {
        local_goal = goal_pose_;
        // local_goal = path.back();
        if_localgoal_final_reached = true;
    }

    // for rviz visualization
    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header.frame_id = "map";
    pos_msg.header.stamp = ros::Time::now();
    pos_msg.pose.position.x = local_goal.x_;
    pos_msg.pose.position.y = local_goal.y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, local_goal.theta_);
    pos_msg.pose.orientation.x = q.x();
    pos_msg.pose.orientation.y = q.y();
    pos_msg.pose.orientation.z = q.z();
    pos_msg.pose.orientation.w = q.w();
    localgoalPub_.publish(pos_msg);
    return local_goal;
}

std::vector<RobotState> pathTracker::orientationFilter(std::vector<RobotState> origin_path)
{
    std::vector<RobotState> path;
    double init_theta = cur_pose_.theta_;
    double goal_theta = goal_pose_.theta_;
    double theta_err = 0;
    double d_theta = 0;
    Eigen::Vector3d init;
    Eigen::Vector3d goal;
    // calculate rotate direction
    init << cos(init_theta), sin(init_theta), 0;
    goal << cos(goal_theta), sin(goal_theta), 0;

    if (init.cross(goal)(2) >= 0)
        rotate_direction_ = 1;
    else
        rotate_direction_ = -1;

    // theta_err = acos(init(0)*goal(0)+init(1)*goal(1));
    theta_err = fabs(angleLimitChecking(goal_theta - init_theta));
    d_theta = rotate_direction_ * theta_err / (origin_path.size() - 1);

    RobotState point(origin_path.at(0).x_, origin_path.at(0).y_, init_theta);
    path.push_back(point);

    for (int i = 0; i < origin_path.size(); i++)
    {
        if (i != 0)
        {
            double theta;
            theta = angleLimitChecking(path.at(i - 1).theta_ + d_theta);
            // cout << "theta = " << theta << endl;
            RobotState point(origin_path.at(i).x_, origin_path.at(i).y_, theta);
            path.push_back(point);
        }
    }

    // Rviz visualize processed path
    geometry_msgs::PoseArray arr_msg;
    arr_msg.header.frame_id = "map";
    arr_msg.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Pose> poses;

    for (int i = 0; i < path.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = path.at(i).x_;
        pose.position.y = path.at(i).y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, path.at(i).theta_);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        poses.push_back(pose);
    }
    arr_msg.poses = poses;
    posearrayPub_.publish(arr_msg);

    return path;
}

double pathTracker::angleLimitChecking(double theta)
{
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    return theta;
}

// Path tracker for differential drive robot
void pathTracker::diffController(RobotState local_goal, RobotState cur_pos)
{
}

// Path tracker for omni drive robot
void pathTracker::omniController(RobotState local_goal, RobotState cur_pos)
{
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    int rotate_direction = 0;
    Eigen::Vector3d goal_vec(goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);
    Eigen::Vector3d cur_vec(cur_pos.x_, cur_pos.y_, cur_pos.theta_);

    if (cur_vec.cross(goal_vec)(2) >= 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;

    // transform local_goal to base_footprint frame
    Eigen::Vector2d goal_base_vec;
    Eigen::Vector2d localgoal_bf;
    Eigen::Matrix2d rot;
    goal_base_vec << (local_goal.x_ - cur_pos.x_), (local_goal.y_ - cur_pos.y_);
    rot << cos(-cur_pos.theta_), -sin(-cur_pos.theta_), sin(-cur_pos.theta_), cos(-cur_pos.theta_);
    localgoal_bf = rot * goal_base_vec;

    if (xy_goal_reached(cur_pose_, goal_pose_))
    {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
    }
    else
    {
        linear_velocity =
            velocityProfile(Velocity::linear, cur_pose_, goal_pose_, velocity_state_, linear_acceleration_);
        double direction = atan2(localgoal_bf(1), localgoal_bf(0));
        velocity_state_.x_ = linear_velocity * cos(direction);
        velocity_state_.y_ = linear_velocity * sin(direction);
    }

    if (theta_goal_reached(cur_pose_, goal_pose_))
    {
        velocity_state_.theta_ = 0;
    }
    else
    {
        angular_velocity =
            velocityProfile(Velocity::angular, cur_pose_, local_goal, velocity_state_, angular_acceleration_);
        velocity_state_.theta_ = angular_velocity;
    }
    velocityPublish();
}

double pathTracker::velocityProfile(Velocity vel_type, RobotState cur_pos, RobotState goal_pos, RobotState vel_state_,
                                    double acceleration)
{
    double output_vel = 0;
    if (workingMode_ == Mode::TRACKING)
    {
        if (vel_type == Velocity::linear)
        {
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            double xy_err = cur_pose_.distanceTo(goal_pose_);

            // acceleration
            if (xy_err > linear_brake_distance_)
            {
                if (linear_acceleration_profile_ == "linear")
                {
                    // ROS_INFO_THROTTLE(1,"linear acc");
                    double d_vel = acceleration / control_frequency_;
                    output_vel = last_vel + d_vel;
                    max_linear_vel_reached_ = output_vel;
                }
                else if (linear_acceleration_profile_ == "smooth_step")
                {
                }
            }
            // deceleration
            else
            {
                // ROS_INFO_THROTTLE(1,"err = %f\n", xy_err);
                if (linear_deceleration_profile_ == "linear")
                {
                    // double acc = pow(linear_max_vel_, 2) / 2 / linear_brake_distance_;
                    double acc = pow(max_linear_vel_reached_, 2) / 2 / linear_brake_distance_;
                    output_vel = sqrt(2 * acc * xy_err);
                    // ROS_INFO_THROTTLE(1,"max_linear_vel_reached_ = %f", max_linear_vel_reached_);
                    // ROS_INFO_THROTTLE(1,"acc = %f", acc);
                    // ROS_INFO_THROTTLE(1,"output_vel = %f", output_vel);
                    // if (output_vel < 0.4)
                    if (cur_pose_.distanceTo(goal_pose_) < 0.15)
                    {
                        double p_vel = xy_err * linear_kp_;

                        // if (p_vel < output_vel)
                        if (1)
                        {
                            output_vel = p_vel;
                        }
                    }
                }
                else if (linear_deceleration_profile_ == "p_control")
                {
                    output_vel = cur_pos.distanceTo(goal_pos) * linear_kp_;
                }
                else if (linear_deceleration_profile_ == "smooth_step")
                {
                }
            }

            // Saturation
            if (output_vel > linear_max_vel_)
                output_vel = linear_max_vel_;
            // ROS_INFO_THROTTLE(1,"linear vel %f", output_vel);
        }

        if (vel_type == Velocity::angular)
        {
            // double theta_err;
            // // theta_err = fabs(angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));
            // theta_err = (angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));
            // output_vel = theta_err * angular_kp_;

            // // if (signbit(acceleration))
            // // {
            // //     output_vel *= -1;
            // // }
            // // Saturation
            // if (output_vel > angular_max_vel_)
            //     output_vel = angular_max_vel_;
            // if (output_vel < -angular_max_vel_)
            //     output_vel = -angular_max_vel_;

            // ================= old version =================
            double d_vel = acceleration / control_frequency_;
            double theta_err = (angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));

            output_vel = vel_state_.theta_ + d_vel * theta_err / fabs(theta_err);

            if (fabs(theta_err) < angular_brake_distance_)
            {
                output_vel = theta_err * angular_kp_;
            }

            // Saturation
            if (output_vel > angular_max_vel_)
                output_vel = angular_max_vel_;
            if (output_vel < -angular_max_vel_)
                output_vel = -angular_max_vel_;
        }
    }

    else if (workingMode_ == Mode::TRANSITION)
    {
        if (vel_type == Velocity::linear)
        {
            double d_vel = linear_transition_acc_ / control_frequency_;
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            output_vel = last_vel - d_vel;
            if (output_vel < linear_transition_vel_)
                output_vel = linear_transition_vel_;
        }

        if (vel_type == Velocity::angular)
        {
            double d_vel = angular_transition_acc_ / control_frequency_;
            if (output_vel > 0)
            {
                output_vel = vel_state_.theta_ - d_vel;
                if (output_vel < angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            }
            else
            {
                output_vel = vel_state_.theta_ + d_vel;
                if (output_vel > angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            }
        }
    }

    return output_vel;
}

bool pathTracker::xy_goal_reached(RobotState cur_pos, RobotState goal_pos)
{
    if (cur_pos.distanceTo(goal_pos) < xy_tolerance_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool pathTracker::theta_goal_reached(RobotState cur_pos, RobotState goal_pos)
{
    double theta_err = 0;
    Eigen::Vector2d cur_vec;
    Eigen::Vector2d goal_vec;
    cur_vec << cos(cur_pos.theta_), sin(cur_pos.theta_);
    goal_vec << cos(goal_pos.theta_), sin(goal_pos.theta_);
    theta_err = cur_vec.dot(goal_vec);

    theta_err = fabs(angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));
    if (fabs(theta_err) < theta_tolerance_)
    {
        return true;
    }
    else
        return false;
}

void pathTracker::velocityPublish()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = velocity_state_.x_;
    vel_msg.linear.y = velocity_state_.y_;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = velocity_state_.theta_;
    velPub_.publish(vel_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathTracker_main_vl53");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    pathTracker pathTracker_inst(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
}
