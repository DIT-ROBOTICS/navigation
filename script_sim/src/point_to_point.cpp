#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include "yaml-cpp/yaml.h"
#include "std_msgs/Float64.h"
#include <iostream>

double check = 1;


void Check(const std_msgs::Float64::ConstPtr& msg)
{
    if(msg == 0){
        check=1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_to_point");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("goal_topic", 10);
    ros::Subscriber sub = nh.subscribe("check_topic",10,Check);
    ros::Rate loop_rate(1);
    
    geometry_msgs::Point goal_point;
    YAML::Node pathConfig = YAML::LoadFile("/home/drychang/winter_ws/src/navigation/script_sim/config/script_sim.yaml");
    // YAML::Node pathConfig = YAML::LoadFile("-d $(find script_sim)/config/script_sim.yaml");
    while(ros::ok())
    {
        // if(action==1){
            for(auto goal : pathConfig){
                goal_point.x = goal["xyz"][0].as<double>();
                goal_point.y = goal["xyz"][1].as<double>();
                goal_point.z = goal["xyz"][2].as<double>();
                pub.publish(goal_point);
                // action=0;
                // cout <<"x"<<goal["xyz"].as<double>()<< end1;
                // cout<<"x"<<goal["xyz"].as<double>()<<end1;
                // cout<<"x"<<goal["xyz"].as<double>()<<end1;
            }
        // }
        ros::spinOnce();
       loop_rate.sleep();
        
    }
    return 0;
}