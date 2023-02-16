#include <ros/ros.h>                             
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
// #include <dynamic_tutorials/TutorialsConfig.h>

#define PI 3.1415926

double v, theta, w;



int main(int argc, char** argv){
    ros::init(argc, argv, "omni");     // 初始化hello_cpp_node
    ros::NodeHandle nh, private_nh("~");                     // node 的 handler
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    double vx, vy;
    double _ = 0.0;

    private_nh.param("v", v, _);
    private_nh.param("theta", theta, _);
    private_nh.param("w", w, _);
    geometry_msgs::Twist vel;

    ROS_INFO("v = %f, theta = %f, w = %f", v, theta, w);
    vx = v * cos((double) theta / 180 * PI);
    vy = v * sin((double) theta / 180 * PI);
    vel.linear.x = vx;
    vel.linear.y = vy;
    vel.angular.z = w;
    

    while (ros::ok()){                           
        vel_pub.publish(vel);
        ros::Duration(1).sleep();     
    }
}