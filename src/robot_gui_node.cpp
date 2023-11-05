#include "robot_gui/robot_gui.h"
#include "ros/ros.h"

#define WINDOW_NAME "RobotUI"
int main(int argc, char **argv){
    ros::init(argc,argv,"robot_gui_node");
    RobotGui gui;
    gui.run();
    return 0;
}