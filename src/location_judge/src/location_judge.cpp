// Basic
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
// ROS related
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>



using namespace std;

double pose_x;
double pose_y;

char room_number;

void judge_room(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_x = msg->pose.position.x;
    pose_y = msg->pose.position.y;
    if (pose_x > 5)
    {
        room_number = 'D';
    }
    else
    {
        if (-6.7 > pose_y && pose_y > -13.7)
        {
            room_number = 'C';
        }
        else if (-3.6 > pose_y && pose_y > -6.7)
        {
            room_number = 'B';
        }
        else
        {
            room_number = 'A';
        }
        
    }
} 


int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "location_judge");

    ros::NodeHandle n, nh;
    // geometry_msgs::Twist velCommand;

    ros::Rate loop_rate(10);
    // ros::Publisher  pub = nh.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 3);
    ros::Subscriber sub = n.subscribe("/slam_out_pose", 1, judge_room);

    while(ros::ok())
    {
        ROS_INFO("Pose_x : %f,    Pose_y : %f", pose_x, pose_y);
        ROS_INFO("The robot is in Room: %c", room_number);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} 


