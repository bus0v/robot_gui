#include "robot_gui/robot_gui.h"

RobotGui::RobotGui() {
  ros::NodeHandle nh;

  odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 2,
                                              &RobotGui::odomCallback, this);
  info_sub = nh.subscribe<robot_info::info>("robot_info", 10,
                                            &RobotGui::robotInfoCallback, this);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

void RobotGui::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  data_odom = *msg;
}

void RobotGui::robotInfoCallback(const robot_info::info::ConstPtr &msg) {
  data_info = *msg;
}

void RobotGui::run() {
  cvui::init(WINDOW_NAME);
  cv::Mat frame = cv::Mat(cv::Size(400, 400), CV_8UC3);
  while (ros::ok()) {

    frame = cv::Scalar(49, 52, 49);

    cvui::text(frame, 10, 15, "This is gonna be a great AUV gui!");

    // General Info Area: include an area to display the messages published into
    // the robot_info topic. If the published message changes, the GUI must
    // update accordingly. Commit your work after having created the GUI with
    // the general info area.

    // Teleoperation Buttons: Include buttons for increasing and decreasing the
    // speed in the x-axis direction and the rotation on the z-axis. These
    // buttons must be fully functional, so please ensure that pressing the
    // buttons and modifying the speed updates the data in the cmd_vel topic and
    // the simulated robot behaves accordingly. Commit your work after having
    // created the GUI with the teleoperation buttons.
    if (cvui::button(frame, 100, 50, "Forward")) {
      data_speed.linear.x = data_speed.linear.x + linear_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }
    if (cvui::button(frame, 100, 70, "Backward")) {
      data_speed.linear.x = data_speed.linear.x - linear_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }

    if (cvui::button(frame, 100, 30, "Stop")) {
      data_speed.linear.x = 0.0;
      data_speed.angular.z = 0.0;
      cmd_vel_pub.publish(data_speed);
    }

    if (cvui::button(frame, 30, 50, " Left ")) {
      // The button was clicked, update the Twist message
      data_speed.angular.z = data_speed.angular.z + angular_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }

    // Show a button at position x = 195, y = 50
    if (cvui::button(frame, 195, 50, " Right ")) {
      // The button was clicked, update the Twist message
      data_speed.angular.z = data_speed.angular.z - angular_velocity_step;
      cmd_vel_pub.publish(data_speed);
    }

    // Create window at (320, 20) with size 120x40 (width x height) and title
    cvui::window(frame, 320, 20, 120, 40, "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 345, 45, 0.4, 0xff0000, "%.02f m/sec",
                 data_speed.linear.x);

    // Create window at (320 60) with size 120x40 (width x height) and title
    cvui::window(frame, 320, 60, 120, 40, "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 345, 85, 0.4, 0xff0000, "%.02f rad/sec",
                 data_speed.angular.z);
    // Current velocities: Display the current speed send to the robot via the
    // cmd_vel topic as "Linear Velocity" and "Angular Velocity". These buttons
    // must be fully functional, so please ensure that pressing the buttons
    // modifies the speed shown in this part of the GUI. Commit your work after
    // having created the GUI with the Current velocities.

    // Robot position (Odometry based): Subscribe to /odom and display the
    // current x,y,z position to the GUI. The values displayed must be the
    // actual values being published to the /odom topic. Commit your work after
    // having created the GUI that shows the robot position.

    // Distance travelled service: Create a button that calls the /get_distance
    // service and displays the response message to the screen. Ensure that
    // pressing this button results in a functional service call via te ROS
    // network and the response from the distance_tracker_service node is
    // displayed in the GUI. The button's full functionality is required. Commit
    // your work after having completed this part.
    if (cvui::button(frame, 100, 50, "Get Distance")) {
    }
    cvui::update();

    cvui::imshow(WINDOW_NAME, frame);
    if (cv::waitKey(20) == 27) {
      break;
    }
  }
}