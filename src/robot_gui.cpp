#include "robot_gui/robot_gui.h"

RobotGui::RobotGui() {
  ros::NodeHandle nh;

  odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 2,
                                              &RobotGui::odomCallback, this);
  info_sub = nh.subscribe<robot_info::info>("robot_info", 10,
                                            &RobotGui::robotInfoCallback, this);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::service::waitForService("/get_distance");
  distanceClient = nh.serviceClient("/get_distance");
}

void RobotGui::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  data_odom = *msg;
}

void RobotGui::robotInfoCallback(const robot_info::info::ConstPtr &msg) {
  data_info = *msg;
}

void RobotGui::run() {
  cvui::init(WINDOW_NAME);
  cv::Mat frame = cv::Mat(cv::Size(400, 500), CV_8UC3);
  while (ros::ok()) {
    int buttonSizeX = 90;
    int buttonSizeY = 90;
    int incrementX = 100;
    int incrementY = 100;
    frame = cv::Scalar(49, 52, 49);
    // General Info Area: include an area to display the messages published into
    // the robot_info topic. If the published message changes, the GUI must
    // update accordingly. Commit your work after having created the GUI with
    // the general info area.
    cvui::window(frame, 10, 15, buttonSizeX * 0.9, buttonSizeY * 2,
                 "General Info");
    // Teleoperation Buttons: Include buttons for increasing and decreasing the
    // speed in the x-axis direction and the rotation on the z-axis. These
    // buttons must be fully functional, so please ensure that pressing the
    // buttons and modifying the speed updates the data in the cmd_vel topic and
    // the simulated robot behaves accordingly. Commit your work after having
    // created the GUI with the teleoperation buttons.
    if (cvui::button(frame, incrementX * 2, incrementY, buttonSizeX,
                     buttonSizeY, "Forward")) {

      data_speed.linear.x = data_speed.linear.x + linear_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }
    if (cvui::button(frame, incrementX * 2, incrementY * 3, buttonSizeX,
                     buttonSizeY, "Backward")) {
      data_speed.linear.x = data_speed.linear.x - linear_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }

    if (cvui::button(frame, incrementX * 2, incrementY * 2, buttonSizeX,
                     buttonSizeY, "Stop")) {
      data_speed.linear.x = 0.0;
      data_speed.angular.z = 0.0;
      cmd_vel_pub.publish(data_speed);
    }

    if (cvui::button(frame, incrementX, incrementY * 2, buttonSizeX,
                     buttonSizeY, " Left ")) {
      // The button was clicked, update the Twist message
      data_speed.angular.z = data_speed.angular.z + angular_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }

    // Show a button at position x = 195, y = 50
    if (cvui::button(frame, incrementX * 3, incrementY * 2, buttonSizeX,
                     buttonSizeY, " Right ")) {
      // The button was clicked, update the Twist message
      data_speed.angular.z = data_speed.angular.z - angular_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }
    // Current velocities: Display the current speed send to the robot via the
    // cmd_vel topic as "Linear Velocity" and "Angular Velocity". These buttons
    // must be fully functional, so please ensure that pressing the buttons
    // modifies the speed shown in this part of the GUI. Commit your work after
    // having created the GUI with the Current velocities.

    // Create window at (320, 20) with size 120x40 (width x height) and title
    cvui::window(frame, incrementX, 20, buttonSizeX * 1.5, 70,
                 "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, incrementX, 45, 0.4, 0xff0000, "%.02f m/sec",
                 data_speed.linear.x);

    // Create window at (320 60) with size 120x40 (width x height) and title
    cvui::window(frame, incrementX * 2.5, 20, buttonSizeX * 1.5, 70,
                 "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, incrementX * 3, 45, 0.4, 0xff0000, "%.02f rad/sec",
                 data_speed.angular.z);
    // Robot position (Odometry based): Subscribe to /odom and display the
    // current x,y,z position to the GUI. The values displayed must be the
    // actual values being published to the /odom topic. Commit your work after
    // having created the GUI that shows the robot position.
    cvui::window(frame, incrementX, incrementY * 4, buttonSizeX, buttonSizeY,
                 "X Position: ");
    cvui::printf(frame, incrementX, incrementY * 4 + 20, 0.4, 0xff0000, "%.02f m",
                 data_odom.pose.pose.position.x);

    cvui::window(frame, incrementX * 2, incrementY * 4, buttonSizeX,
                 buttonSizeY, "Y Position: ");
    cvui::printf(frame, incrementX*2, incrementY * 4 +20, 0.4, 0xff0000, "%.02f m",
                 data_odom.pose.pose.position.y);

    cvui::window(frame, incrementX * 3, incrementY * 4, buttonSizeX,
                 buttonSizeY, "Z Position");
    cvui::printf(frame, incrementX*3, incrementY * 4 +20, 0.4, 0xff0000, "%.02f m",
                data_odom.pose.pose.position.z);

    // Distance travelled service: Create a button that calls the /get_distance
    // service and displays the response message to the screen. Ensure that
    // pressing this button results in a functional service call via te ROS
    // network and the response from the distance_tracker_service node is
    // displayed in the GUI. The button's full functionality is required. Commit
    // your work after having completed this part.
    if (cvui::button(frame, 10, incrementY * 2, buttonSizeX - 10, buttonSizeY,
                     "Get Distance")) {
    }
    cvui::window(frame, 10, incrementY * 3, buttonSizeX * 0.9, buttonSizeY * 2,
                 "Distance Travelled");
    cvui::update();

    cvui::imshow(WINDOW_NAME, frame);
    if (cv::waitKey(20) == 27) {
      break;
    }
  }
}