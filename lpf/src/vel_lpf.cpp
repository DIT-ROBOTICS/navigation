#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

double beta = 0.0;
double vx[3] = {0.0};
double vy[3] = {0.0};
double vth[3] = {0.0};
double publish_frequency = 50.0;
// double last_time;
// double now_time;
// double dt = 0.0;
// int count = 0;

// void timerCallback(const ros::TimerEvent& event){
  
// }

void lpf_callback(const geometry_msgs::Twist& data){
  vx[1] = data.linear.x;
  vy[1] = data.linear.y;
  vth[1] = data.angular.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "vel_lpf");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  nh_local.param<double>("beta", beta, 0.2);
  nh_local.param<double>("publish_frequency", publish_frequency, 50.0);

  ros::Publisher lpf_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub = nh.subscribe("raw_cmd_vel", 1, lpf_callback);

  ros::Rate r(publish_frequency);

  while(ros::ok()){
    //Simple low-pass filter
    vx[2] = beta*vx[1] + (1-beta)*vx[0];
    vy[2] = beta*vy[1] + (1-beta)*vy[0];
    vth[2] = beta*vth[1] + (1-beta)*vth[0];

    // ROS_INFO("%f %f\n", (ros::Time::now()).toSec(), vx[1]);
    // ROS_INFO("vy:\t%f\n", vy[1]);
    // ROS_INFO("vth:\t%f\n\n", vth[1]);

    //V(n-1) = V(n)
    vx[0] = vx[2];
    vy[0] = vy[2];
    vth[0] = vth[2];

    // now_time = ros::Time::now().toSec();
    // dt = now_time - last_time;
    // ROS_INFO("hz: %f times: %d", 1/dt, count++);
    // last_time = ros::Time::now().toSec();

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx[2];
    cmd_vel.linear.y = vy[2];
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = vth[2];
    lpf_pub.publish(cmd_vel);

    ros::spinOnce();
    r.sleep();
  }
}