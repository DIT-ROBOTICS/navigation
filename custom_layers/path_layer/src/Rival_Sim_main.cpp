// ros
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// other
#include "path_layer/RobotClass.h"

using namespace _ROBOT_CLASS_;

static ROBOT_STATE Rival;
static geometry_msgs::Point GoalPoint[2];

void GeneratePath();

int main(int argc, char** argv) {
    ros::init(argc, argv, "Robot_Sim");

    ros::NodeHandle nh;
    // ros::Publisher RivalPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("RivalPose", 100);
    ros::Publisher RivalPath_Pub = nh.advertise<nav_msgs::Path>("RivalPath", 100);

    Rival.SetRobotType(_ROBOT_CLASS_::ROBOT_TYPE::Rival);
    GoalPoint[0].x = 0.6;
    GoalPoint[0].y = 1.0;
    GoalPoint[1].x = 2.3;
    GoalPoint[1].y = 1.0;

    ros::Rate LoopRate(15.0);
    while (nh.ok()) {
        ros::spinOnce();

        if (!Rival.GoToNextPoint()) {
            GeneratePath();
        }

        // geometry_msgs::PoseStamped CurRivalPose;
        nav_msgs::Path RivalPath_msgs;

        // // header
        // CurRivalPose.header.frame_id = "map";
        // CurRivalPose.header.stamp = ros::Time::now();

        // // Pose
        // CurRivalPose.pose.position.x = Rival.GetPosition().position.x;
        // CurRivalPose.pose.position.y = Rival.GetPosition().position.y;
        // CurRivalPose.pose.orientation.z = Rival.GetPosition().orientation.z;
        // CurRivalPose.pose.orientation.w = Rival.GetPosition().orientation.w;

        // RivalPose_Pub.publish(CurRivalPose);

        // Header
        RivalPath_msgs.header.frame_id = "map";
        RivalPath_msgs.header.stamp = ros::Time::now();

        // Path
        std::queue<geometry_msgs::Pose> Temp_Path(Rival.Path);
        while (!Temp_Path.empty()) {
            geometry_msgs::PoseStamped temp;

            temp.pose = Temp_Path.front();
            temp.header.frame_id = "map";
            temp.header.stamp = ros::Time::now();
            RivalPath_msgs.poses.push_back(temp);

            Temp_Path.pop();
        }

        RivalPath_Pub.publish(RivalPath_msgs);

        LoopRate.sleep();
    }

    return 0;
}

void GeneratePath() {
    std::queue<geometry_msgs::Pose> GoalPath;

    if (Rival.isReach(GoalPoint[0])) {
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

    Rival.SetPath(GoalPath);

    ros::Duration(2.0).sleep();
}