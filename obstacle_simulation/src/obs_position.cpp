#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

void obstaclePub(ros::Publisher pub, std::string frame, std::vector<std::vector<double>> obstacle_pos){
    int obstacle_num = obstacle_pos.size();
    for(int i = 0; i < obstacle_num; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame; 
        pose.pose.position.x = obstacle_pos[i][0];
        pose.pose.position.y = obstacle_pos[i][1];
        pub.publish(pose);
    }
}

void obstaclePubArray(ros::Publisher pub, std::string frame, std::vector<std::vector<double>> obstacle_pos){
    int obstacle_num = obstacle_pos.size();
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = frame;
    poses.header.stamp = ros::Time::now();

    for(int i = 0; i < obstacle_num; i++)
    {     
        geometry_msgs::Pose p;
        p.position.x = obstacle_pos[i][0];
        p.position.y = obstacle_pos[i][1];
        poses.poses.push_back(p);
    }
    pub.publish(poses);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_position");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher obs_pub_PoseStamped = nh.advertise<geometry_msgs::PoseStamped>("obstacle_position",1000);
    ros::Publisher obs_pub_PoseArray = nh.advertise<geometry_msgs::PoseArray>("obstacle_position_array",1000);

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
    // std::cout << filename<<std::endl;
    // file.open(filename, std::ios::in);

    // std::string line;
    // getline(file, line);
    // if(file.is_open())
    // {   
    //     ROS_INFO("successfully read csv file");
    //     // while(getline(file, line))
    //     // {
    //         // cin >> obstacle_pos[obstacle_num][0] >> obstacle_pos[obstacle_num][1];
    //         // obstacle num++;
    //     // }
    // }
    // else
    // {
    //     ROS_ERROR("failed to open csv file");
    // }

    while(ros::ok())
    {
        obstaclePub(obs_pub_PoseStamped, frame, obstacle_pos);
        obstaclePubArray(obs_pub_PoseArray, frame, obstacle_pos);
        ros::spinOnce();
        loop_rate.sleep();
    }

}