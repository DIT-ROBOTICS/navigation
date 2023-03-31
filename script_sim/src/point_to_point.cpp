#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "yaml-cpp/yaml.h"

using namespace std;

bool check = true;

geometry_msgs::PoseStamped goal_point;
YAML::Node pathConfig;
int path_length;
tf2::Quaternion qt;

void Check(const std_msgs::Bool::ConstPtr& msg) {
    check = true;
    if (!msg->data) {
        ROS_INFO("script_sim: cannot arrive the goal, next point!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_to_point");
    ros::NodeHandle nh;

    string PathName(argv[1]);
    string PubName(argv[2]);
    string SubName(argv[3]);

    pathConfig = YAML::LoadFile(PathName);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(PubName, 100);
    ros::Subscriber sub = nh.subscribe(SubName, 100, Check);

    ros::Duration(2).sleep();

    ros::Rate loop_rate(1);

    for (auto goal : pathConfig) {
        if (!ros::ok()) {
            break;
        }
        string goal_type = goal["xyz"][0].as<string>();
        // std::cout << goal_type << std::endl;
        if(goal_type == "path") {
            goal_point.header.frame_id = "path";
        }else if(goal_type == "dock"){
            goal_point.header.frame_id = "dock";
        }
        goal_point.header.frame_id = goal_type;
        goal_point.pose.position.x = goal["xyz"][1].as<double>();
        goal_point.pose.position.y = goal["xyz"][2].as<double>();
        qt.setRPY(0, 0, goal["xyz"][3].as<double>());

        goal_point.pose.orientation.x = qt.x();
        goal_point.pose.orientation.y = qt.y();
        goal_point.pose.orientation.z = qt.z();
        goal_point.pose.orientation.w = qt.w();

        pub.publish(goal_point);

        check = false;
        while (!check && ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}