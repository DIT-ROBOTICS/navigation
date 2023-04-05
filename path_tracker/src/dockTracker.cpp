#include "dockTracker.h"

DockTracker::DockTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &DockTracker::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

DockTracker::~DockTracker()
{
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("linear_max_velocity");
    nh_local_.deleteParam("profile_percent");
    nh_local_.deleteParam("stop_tolerance");
}

void DockTracker::initialize()
{
    // zeroing the arrays
    goal_[0] = 0.0;
    goal_[1] = 0.0;
    goal_[2] = 0.0;

    pose_[0] = 0.0;
    pose_[1] = 0.0;
    pose_[2] = 0.0;

    vel_[0] = 0.0;
    vel_[1] = 0.0;
    vel_[2] = 0.0;

    dock_dist_ = 0.05;
    if_get_goal_ = false;
    count_dock_dist_ = false;
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &DockTracker::timerCB, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();
}

bool DockTracker::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    // Load parameter
    bool get_param_ok = true;
    bool prev_active = p_active_;
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    // get_param_ok = nh_local_.param<string>("", _, "");
    get_param_ok = nh_local_.param<double>("control_frequency", control_frequency_, 50);
    get_param_ok = nh_local_.param<double>("linear_max_velocity", linear_max_vel_, 0.3);
    get_param_ok = nh_local_.param<double>("profile_percent", profile_percent_, 0.2);
    get_param_ok = nh_local_.param<double>("stop_tolerance", tolerance_, 0.005);
    get_param_ok = nh_local_.param<int>("odom_type", odom_type_, 0);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            goal_sub_ = nh_.subscribe("dock_goal", 50, &DockTracker::goalCB, this);
            if(odom_type_ == 0)
            {
                pose_sub_ = nh_.subscribe("odom", 50, &DockTracker::poseCB_Odometry, this);
            }else if(odom_type_ == 1){
                pose_sub_ = nh_.subscribe("ekf_pose", 50, &DockTracker::poseCB_PoseWithCovarianceStamped, this);
            }
            pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            goalreachedPub_ = nh_.advertise<std_msgs::Char>("finishornot", 1);
        }
        else
        {
            goal_sub_.shutdown();
            pose_sub_.shutdown();
            pub_.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Docking Tracker]: "
                        << "Set params ok");
    }
    else
    {
        ROS_WARN_STREAM("[Docking Tracker]: "
                        << "Set params failed");
    }
    return true;
}

double DockTracker::distance(double x1, double y1, double x2, double y2)
{
    double distance = 0.0;
    distance = hypot((x2 - x1), (y2 - y1));
    return distance;
}

void DockTracker::velocityPUB()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel_[0];
    cmd_vel.linear.y = vel_[1];
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = vel_[2];
    pub_.publish(cmd_vel);
}

void DockTracker::timerCB(const ros::TimerEvent& e)
{
    // simple p control
    if (if_get_goal_)
    {
        // calculate current distance to dock goal
        dist_ = distance(pose_[0], pose_[1], goal_[0], goal_[1]);

        // robot coordinate's inner product to cosx_ and deal with signs
        cosx_ = ((goal_[0] - pose_[0]) * cos(pose_[2]) + (goal_[1] - pose_[1]) * sin(pose_[2])) / dist_;
        sinx_ = sqrt(1 - pow(cosx_, 2));
        if ((cos(pose_[2]) * (goal_[1] - pose_[1])) - (sin(pose_[2]) * (goal_[0] - pose_[0])) < 0)
            sinx_ *= -1;

        // remember docking distance
        if (!count_dock_dist_)
        {
            dock_dist_ = dist_;
            a_ = pow(linear_max_vel_, 2) / (2 * profile_percent_ * dock_dist_);
            count_dock_dist_ = true;
        }

        // check if profile cut-off point valid
        if (profile_percent_ > 0.5 || profile_percent_ < 0.0)
        {
            profile_percent_ = 0.5;
            // ROS_INFO("[Dock Tracker]: Profile percent out of range, using 0.5!");
        }

        // current time when going in this loop
        t_now_ = ros::Time::now().toSec();
        double dt = t_now_ - t_bef_;

        // velocity profile: trapezoidal
        // accelerate
        if (dist_ > (1 - profile_percent_) * dock_dist_)
        {
            if (fabs(hypot(vel_[0], vel_[1])) >= linear_max_vel_)
            {
                vel_[0] = linear_max_vel_ * cosx_;
                vel_[1] = linear_max_vel_ * sinx_;
                vel_[2] = 0.0;
            }
            else
            {
                vel_[0] = vel_[0] + a_ * dt * cosx_;
                vel_[1] = vel_[1] + a_ * dt * sinx_;
                vel_[2] = 0.0;
            }
            // ROS_INFO("[Dock Tracker]: Accelerate!(v, dist_): %f %f", hypot(vel_[0], vel_[1]), dist_);
        }
        // uniform velocity
        else if (dist_ <= (1 - profile_percent_) * dock_dist_ && dist_ >= profile_percent_ * dock_dist_)
        {
            vel_[0] = linear_max_vel_ * cosx_;
            vel_[1] = linear_max_vel_ * sinx_;
            vel_[2] = 0.0;
            // ROS_INFO("[Dock Tracker]: Uniform Velocity!(v, dist_): %f %f", hypot(vel_[0], vel_[1]), dist_);
        }
        // deccelerate
        else if (dist_ < profile_percent_ * dock_dist_)
        {
            vel_[0] = linear_max_vel_ * (dist_ / (profile_percent_ * dock_dist_)) * cosx_;  // vel_[0]-a_*dt*cosx_;
            vel_[1] = linear_max_vel_ * (dist_ / (profile_percent_ * dock_dist_)) * sinx_;  // vel_[1]-a_*dt*sinx_;
            vel_[2] = 0.0;
            // ROS_INFO("[Dock Tracker]: Deccelerate!(v, dist_): %f %f", hypot(vel_[0], vel_[1]), dist_);
        }
        // error message
        else
        {
            // ROS_WARN("[Dock Tracker]: Docking distance goes wrong(dist_, px, py, gx, gy): %f %f %f %f %f"
            //         , dist_, pose_[0], pose_[1], goal_[0], goal_[1]);
        }

        // stop
        if (dist_ < tolerance_)
        {
            vel_[0] = 0.0;
            vel_[1] = 0.0;
            vel_[2] = 0.0;
            if_get_goal_ = false;
            // Return finishornot = true!
            // for(int i = 0; i < 5; ++i){
            std_msgs::Char finished;
            finished.data = 1;
            goalreachedPub_.publish(finished);
            // }
            ROS_INFO("[Dock Tracker] : Successfully docked!");
        }
    }
    else
    {
        // ROS_INFO_STREAM_THROTTLE(5, "[Dock Tracker]: No goal received!");
    }

    // publish cmd_vel
    velocityPUB();

    // ROS_INFO("%f %f %f", vel_[0], vel_[1], dt);

    // remember the time when leaving this loop
    t_bef_ = t_now_;
}

void DockTracker::goalCB(const geometry_msgs::PoseStamped& data)
{
    vel_[0] = 0.0;
    vel_[1] = 0.0;
    vel_[2] = 0.0;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    goal_[0] = data.pose.position.x;  // + dock_dist_*cos(yaw);
    goal_[1] = data.pose.position.y;  // + dock_dist_*sin(yaw);
    goal_[2] = yaw;
    if_get_goal_ = true;
    t_bef_ = ros::Time::now().toSec();
    ROS_INFO("[Dock Tracker]: Dock goal received!");
}

void DockTracker::poseCB_Odometry(const nav_msgs::Odometry& data){
    pose_[0] = data.pose.pose.position.x;
    pose_[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_[2] = yaw;
    // ROS_INFO("odom: %f %f", pose_[0], pose_[1]);
}

void DockTracker::poseCB_PoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped& data)
{
    pose_[0] = data.pose.pose.position.x;
    pose_[1] = data.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(data.pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    pose_[2] = yaw;
    // ROS_INFO("odom: %f %f", pose_[0], pose_[1]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dockTracker");
    ros::NodeHandle nh(""), nh_local("~");
    DockTracker dockTracker(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
}