#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/PoseStamped.h"

void obstaclePub(ros::Publisher obs_pub, std::string frame, std::vector<std::vector<double>> obstacle_pos){
    int obstacle_num = obstacle_pos.size();
    for(int i = 0; i < obstacle_num; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame; 
        pose.pose.position.x = obstacle_pos[i][0];
        pose.pose.position.y = obstacle_pos[i][1];
        obs_pub.publish(pose);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_position");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher obs_pub = nh.advertise<geometry_msgs::PoseStamped>("obstacle_position",1000);

    // private parameters
    std::string filename, frame;
    private_nh.param("filename", filename, std::string("filename"));    
    private_nh.param("frame", frame, std::string("map"));
    
    int update_frequency;
    private_nh.param("update_frequency", update_frequency, 10);
    ros::Rate loop_rate(update_frequency);

    
    double obstacle_num = 0;

    std::fstream file;
    std::vector<std::vector<double>> obstacle_pos{
        {1,1},
        {2,1},
        {2.5,1}
    };
    std::cout << filename<<std::endl;
    file.open(filename, std::ios::in);

    std::string line;
    getline(file, line);
    if(file.is_open())
    {   
        ROS_INFO("successfully read csv file");
        // while(getline(file, line))
        // {
            // cin >> obstacle_pos[obstacle_num][0] >> obstacle_pos[obstacle_num][1];
            // obstacle num++;
        // }
    }
    else
    {
        ROS_ERROR("failed to open csv file");
    }

    while(ros::ok())
    {

        ROS_INFO("hello world");

        obstaclePub(obs_pub, frame, obstacle_pos);
        ros::spinOnce();
        loop_rate.sleep();
    }

}