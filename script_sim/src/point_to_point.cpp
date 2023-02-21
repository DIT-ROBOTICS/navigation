#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "yaml-cpp/yaml.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
using namespace std;
double check = 1;


void Check(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == 1){
        check=1;

    }
    ROS_INFO("hello");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_to_point1");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("robot1/nav_goal", 10);
    ros::Subscriber sub = nh.subscribe("robot1/finishornot",10,Check);
    ros::Rate loop_rate(1); 

    geometry_msgs::PoseStamped goal_point;
    YAML::Node pathConfig = YAML::LoadFile("/home/ubuntu/Eurobot2023_ws/src/navigation/script_sim/config/script_sim.yaml");
    // YAML::Node pathConfig = YAML::LoadFile("-d $(find script_sim)/config/script_sim.yaml");
    // 
        loop_rate.sleep();
            for(auto goal : pathConfig){
                if(!ros::ok()){
                    break;
                }
                goal_point.header.frame_id = "robot1/map";
                goal_point.pose.position.x = goal["xyz"][0].as<double>();
                goal_point.pose.position.y = goal["xyz"][1].as<double>();

                tf2::Quaternion qt;
                qt.setRPY(0,0, goal["xyz"][2].as<double>());

                goal_point.pose.orientation.x = qt.x();
                goal_point.pose.orientation.y = qt.y();
                goal_point.pose.orientation.z = qt.z();
                goal_point.pose.orientation.w = qt.w();
                pub.publish(goal_point);
                check=0;
                cout <<"x"<<goal["xyz"][0].as<double>()<< endl;
                cout<<"y"<<goal["xyz"][1].as<double>()<<endl;
                cout<<"z"<<goal["xyz"][2].as<double>()<<endl;
                while(check==0&&ros::ok()){
                    ros::spinOnce();
                    
                }
                loop_rate.sleep();
            }
        
        
    return 0;
}