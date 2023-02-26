// ros
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

// other
#include <cmath>

#include "path_layer/RobotClass.h"

using namespace _ROBOT_CLASS_;

static int RivalNum = 0;
static geometry_msgs::Point GoalPoint[2];
static ROBOT_STATE Rival[2];
static int UpdateFrequency;

void GeneratePath();
void RivalVel_CB(const geometry_msgs::Twist& msg);

int main(int argc, char** argv) {
    ros::init(argc, argv, "Robot_Sim");

    ros::NodeHandle nh("~");

    // Param
    nh.getParam("RivalNum", RivalNum);
    nh.param("UpdateFrequency", UpdateFrequency, 15);

    if (RivalNum == 0) {
        ROS_WARN("RivalNum is 0, turning off simulation ...");
        return EXIT_SUCCESS;
    }

    ros::Publisher RivalPoseArray_Pub = nh.advertise<geometry_msgs::PoseArray>("RivalPoseArray", 100);
    ros::Subscriber RivalPose_Sub = nh.subscribe("/RivalVel", 1000, RivalVel_CB);

    Rival[0].Position.position.x = 1.5;
    Rival[0].Position.position.y = 1.2;
    Rival[0].Position.orientation.z = 0.0;
    Rival[0].Position.orientation.w = 0.0;
    Rival[0].SetRobotType(_ROBOT_CLASS_::ROBOT_TYPE::Rival);
    Rival[1].SetRobotType(_ROBOT_CLASS_::ROBOT_TYPE::Rival);

    GoalPoint[0].x = 0.6;
    GoalPoint[0].y = 1.0;
    GoalPoint[1].x = 2.3;
    GoalPoint[1].y = 1.0;

    ros::Time CurrentTime = ros::Time::now();
    ros::Time LastTime = CurrentTime;

    ros::Rate LoopRate(UpdateFrequency);
    while (nh.ok()) {
        ros::spinOnce();

        CurrentTime = ros::Time::now();
        double dt = (CurrentTime - LastTime).toSec();

        if (!Rival[1].GoToNextPoint()) {
            GeneratePath();
        }

        geometry_msgs::PoseArray RivalPoseArray_msg;

        // header
        RivalPoseArray_msg.header.frame_id = "robot1/map";
        RivalPoseArray_msg.header.stamp = ros::Time::now();

        // Rival[0] Pose
        geometry_msgs::Pose Rival0_Pose;

        // NOTE: Use X as Y below.
        Rival[0].Position.position.x += -Rival[0].Velocity.linear.y * dt;
        Rival[0].Position.position.y += Rival[0].Velocity.linear.x * dt;

        if (Rival[0].Velocity.linear.x != 0.0 || Rival[0].Velocity.linear.y != 0.0) {
            double Yaw;
            if (Rival[0].Velocity.linear.y == 0.0) {
                Yaw = (Rival[0].Velocity.linear.x >= 0) ? M_PI / 2.0 : M_PI * 1.5;
            } else {
                Yaw = atan(Rival[0].Velocity.linear.x / -Rival[0].Velocity.linear.y);
                if (-Rival[0].Velocity.linear.y <= 0) {
                    Yaw += M_PI;
                }
            }
            tf::Quaternion temp = tf::createQuaternionFromYaw(Yaw);
            Rival[0].Position.orientation.z = temp.getZ();
            Rival[0].Position.orientation.w = temp.getW();
        }

        Rival0_Pose.position.x = Rival[0].GetPosition().position.x;
        Rival0_Pose.position.y = Rival[0].GetPosition().position.y;
        Rival0_Pose.orientation.z = Rival[0].GetPosition().orientation.z;
        Rival0_Pose.orientation.w = Rival[0].GetPosition().orientation.w;

        // Rival[1] Pose
        geometry_msgs::Pose Rival1_Pose;
        Rival1_Pose.position.x = Rival[1].GetPosition().position.x;
        Rival1_Pose.position.y = Rival[1].GetPosition().position.y;
        Rival1_Pose.orientation.z = Rival[1].GetPosition().orientation.z;
        Rival1_Pose.orientation.w = Rival[1].GetPosition().orientation.w;

        // Generate Msg
        RivalPoseArray_msg.poses.push_back(Rival0_Pose);
        if (RivalNum == 2) {
            RivalPoseArray_msg.poses.push_back(Rival1_Pose);
        }

        RivalPoseArray_Pub.publish(RivalPoseArray_msg);

        LastTime = CurrentTime;
        LoopRate.sleep();
    }

    return 0;
}

void GeneratePath() {
    std::queue<geometry_msgs::Pose> GoalPath;

    if (Rival[1].isReach(GoalPoint[0])) {
        // Goto GoalPoint[1]
        geometry_msgs::Pose temp;
        tf::Quaternion RivalYaw = tf::createQuaternionFromYaw(0);

        for (double x = GoalPoint[0].x; x < GoalPoint[1].x; x += 0.03) {
            temp.position.x = x;
            temp.position.y = GoalPoint[0].y;
            temp.orientation.z = RivalYaw.getZ();
            temp.orientation.w = RivalYaw.getW();
            GoalPath.push(temp);
        }
    } else {
        // Goto GoalPoint[0]
        geometry_msgs::Pose temp;
        tf::Quaternion RivalYaw = tf::createQuaternionFromYaw(M_PI);

        for (double x = GoalPoint[1].x; x > GoalPoint[0].x; x -= 0.03) {
            temp.position.x = x;
            temp.position.y = GoalPoint[1].y;
            temp.orientation.z = RivalYaw.getZ();
            temp.orientation.w = RivalYaw.getW();
            GoalPath.push(temp);
        }
    }

    Rival[1].SetPath(GoalPath);

    // ros::Duration(2.0).sleep();
}

void RivalVel_CB(const geometry_msgs::Twist& msg) {
    Rival[0].Velocity = msg;
}