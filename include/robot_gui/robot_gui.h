#pragma once
#include "ros/service_client.h"
#include <string>
#define CVUI_IMPLEMENTATION

#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include "robot_info/info.h"
#include "ros/subscriber.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class RobotGui {
public:
  RobotGui();
  void run();

private:
  ros::Subscriber odom_sub;
  ros::Subscriber info_sub;
  ros::Publisher cmd_vel_pub;
  ros::ServiceClient distanceClient;
  std_srvs::Trigger srv;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;
  std::string distanceReceived;
  nav_msgs::Odometry data_odom;
  geometry_msgs::Twist data_speed;
  robot_info::info data_info;
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void robotInfoCallback(const robot_info::info::ConstPtr &msg);
  const std::string WINDOW_NAME = "Robot Gui";
};