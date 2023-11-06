#pragma once
#include "ros/service_client.h"
#define CVUI_IMPLEMENTATION

#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include "robot_gui/cvui.h"
#include "robot_info/info.h"
class RobotGui{
  public:
    RobotGui();
    void run();

  private:
    ros::Subscriber odom_sub;
    ros::Subscriber info_sub;
    ros::Publisher cmd_vel_pub;
    ros::ServiceClient distanceClient;
    float linear_velocity_step = 0.1;
    float angular_velocity_step = 0.1;
    nav_msgs::Odometry data_odom;
    geometry_msgs::Twist data_speed;
    robot_info::info data_info;
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void robotInfoCallback(const robot_info::info::ConstPtr &msg);
    const std::string WINDOW_NAME = "Robot Gui";  
};