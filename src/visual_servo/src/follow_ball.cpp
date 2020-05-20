// Basic
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
// ROS related
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
// CV related
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv/cv.h>  



using namespace std;

float target_x;
float target_y;

void image_process(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat cv_image;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }  
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }  
    (cv_ptr->image).copyTo(cv_image);
    cv::flip(cv_image,cv_image,1);
    cv::Mat cv_image_out;
    cv::blur(cv_image,cv_image_out,cv::Size(10,10));
    imshow("Original Image", cv_image_out);

    cv::Vec3b RGB;
    // Filtering the unrelated color
    for(int i=0;i<cv_image_out.rows;i++){
        for(int j=0;j<cv_image_out.cols;j++){
            RGB=cv_image_out.at<cv::Vec3b>(i,j);
            if((RGB[2]>=160)&&(RGB[1]>=80)&&(RGB[0]<=140)){
                // Here we define yellow with range [R(160~255) G(80~255) B(0~140)]
                cv_image_out.at<cv::Vec3b>(i,j)=cv_image_out.at<cv::Vec3b>(i,j);
            }
            else{
                // The other areas are rendered in black
                cv_image_out.at<cv::Vec3b>(i,j)=0;
            }
        } 
    }
    // Processing the image
    cv::Mat srcImage = cv_image_out;
    cv::Mat midImage, dstImage;
    // Grey image for edge dection.
    cv::cvtColor(srcImage, midImage, CV_BGR2GRAY);
    // Smoothing image.
    cv::GaussianBlur( midImage, midImage, cv::Size(9, 9), 2, 2 );
    // Finally we use the HoughCircles transformation for obtaining the circle
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles( midImage, circles, CV_HOUGH_GRADIENT, 1.5, 10, 200, 100, 0, 0 );


    for( size_t i = 0; i < circles.size(); i++ ){
        target_x=(circles[i][0]);
        target_y=(circles[i][1]);

        cv::Point center(cvRound(target_x), cvRound(target_y));
        int radius = cvRound(circles[i][2]);
        // Draw the circle center
        cv::circle( srcImage, center, 5, cv::Scalar(0,0,255), -1, 8, 0);
        // Draw the circle contour
        cv::circle( srcImage, center, radius, cv::Scalar(255,0,0), 3, 8, 0 );
    }
    cv::imshow("Tracking Ball", srcImage);
    cv::waitKey(3);
} 
  
int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "follow_ball");

    ros::NodeHandle n, nh;
    geometry_msgs::Twist velCommand;

    ros::Rate loop_rate(10);
    ros::Publisher  pub = nh.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 3);
    ros::Subscriber sub = n.subscribe("/vrep/image", 1, image_process);

    while(ros::ok()){
        if( abs(target_y-320) < 80 ) {
            // target_y (240~400) determines linear velocity
            velCommand.linear.x  = 0.8 + (target_y-320)*(-0.03);
            // target_x (  0~512) determines turning left or right
            velCommand.angular.z = 0 + (target_x-256)*(-0.02);
            ROS_INFO("targetX= %f, targetY = %f", target_x, target_y);
            ROS_INFO("Tracking Command: linear= %f, angular = %f", velCommand.linear.x, velCommand.angular.z);
            pub.publish(velCommand);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} 