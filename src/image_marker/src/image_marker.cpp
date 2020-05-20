// Basic
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
// ROS related
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <detection_msgs/Detection.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>

using namespace std;

double pose_x;
double pose_y;
double pose_z;

double quaternion_x;
double quaternion_y;
double quaternion_z;
double quaternion_w;


Eigen::Matrix3d rotation_matrix4;

double image_x;
double image_width;
double center;
double confidence;


double degree;
double depth;

void get_transform_matrix(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_x = msg->pose.position.x;
    pose_y = msg->pose.position.y;
    pose_z = msg->pose.position.z;
    quaternion_x = msg->pose.orientation.x;
    quaternion_y = msg->pose.orientation.y;
    quaternion_z = msg->pose.orientation.z;
    quaternion_w = msg->pose.orientation.w;    
    // rotation_matrix4 = quaternion4.matrix();

} 



void get_image_location(const detection_msgs::Detection& msg){
    image_x = msg.x;
    confidence = msg.confidence;
    image_width = msg.width;
    center = image_x + image_width/2;
}


void get_scan_data(const sensor_msgs::LaserScan& msg){
    degree = 67.5 + center / 512;
    depth = msg.ranges[int(902*degree/180)];
}

int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "location_judge");

    ros::NodeHandle n, nh;

    ros::Rate loop_rate(10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber sub_1 = n.subscribe("/slam_out_pose", 1, get_transform_matrix);
    ros::Subscriber sub_2 = n.subscribe("/facedetector/faces", 1, get_image_location);
    ros::Subscriber sub_3 = n.subscribe("/vrep/scan", 1, get_scan_data);



    while(ros::ok())
    {
        double x;
        double y;

        x = depth * cos((degree/180)*3.1415);
        y = depth * sin((degree/180)*3.1415);

        Eigen::Quaterniond qRot(quaternion_w, quaternion_x, quaternion_y, quaternion_z);
        Eigen::Vector3d vTrans(pose_x, pose_y, 0);
        Eigen::Isometry3d T_body_in_odom = Eigen::Isometry3d::Identity();
        T_body_in_odom.translate(vTrans);
        T_body_in_odom.rotate(qRot);

        Eigen::Vector3d vBody(x,-y,0);
        Eigen::Vector3d vOdom = T_body_in_odom * vBody;

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = vOdom[0];
        marker.pose.position.y = vOdom[1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.5;
        marker.scale.y = 1.0;
        marker.scale.z = 0.5;
    
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 255.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
    
        marker.lifetime = ros::Duration();
        if(confidence != 0)
        {
            marker_pub.publish(marker);
        }
        

        cout << confidence << endl;
        // cout << x << "    " << y << endl;
        // cout << vOdom << endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} 


