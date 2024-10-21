#include "my_rb1_ros/Rotate.h"
#include "my_rb1_ros/RotateRequest.h"
#include "my_rb1_ros/RotateResponse.h"
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class RobotRotate {

public:
  // ROS Objects
  ros::NodeHandle nh_;

  // ROS Services
  ros::ServiceServer my_service;

  // ROS Publishers
  ros::Publisher pub;

  // ROS Subscribers
  ros::Subscriber sub;

  // ROS Messages
  geometry_msgs::Twist move;

  // Helper Variables
  nav_msgs::Odometry odom;
  float yaw_;
  float destination_;

  // Constructor
  RobotRotate() {

    // Initialize Service, Publisher, and Subscriber
    my_service =
        nh_.advertiseService("/rotate_robot", &RobotRotate::service_cb, this);
    ROS_INFO("The Service /rotate_robot is READY");
    pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub = nh_.subscribe("odom", 1000, &RobotRotate::odom_cb, this);
  }

  // Helper Functions
  bool rotate_robot(int deg) {

    // Log Service Called Degrees
    ROS_INFO("Request Data==> degrees=%d", deg);

    // Convert Degrees to Radians
    float radians = deg * M_PI / 180.0;
    this->destination_ = yaw_ + radians;

    // Initalize Error and Turn Direction
    float error = std::abs(this->destination_ - yaw_);
    float sign = 0.0;

    // Turn Direction
    if (deg > 0.0) {
      sign = 1.0;
    } else if (deg < 0.0) {
      sign = -1.0;
    }

    // Turn to service called angle
    while (error > 0.001) {
      error = std::abs(this->destination_ - yaw_);
      move.angular.z = sign * error * 0.5;
      pub.publish(move);
      ros::spinOnce(); // For the subscriber
    }

    // Stop Rotation
    move.angular.z = 0.0;
    pub.publish(move);

    if (error < 0.002) {
      return true;
    } else {
      return false;
    }
  }

  // Callbacks
  void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {

    // Convert Quaternion to Euler
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Assign Odometry Yaw to object yaw member variable
    this->yaw_ = yaw;

    // Log Odometry Yaw
    // ROS_INFO("%f", this->yaw_);
    // ROS_INFO("%s", msg->header.frame_id.c_str());
  }

  bool service_cb(my_rb1_ros::Rotate::Request &req,
                  my_rb1_ros::Rotate::Response &res) {
    ROS_INFO("The Service /rotate_robot has been called");

    if (rotate_robot(req.degrees)) {
      res.result = "Robot Rotated Successfully";
    } else {
      res.result = "Error. Could not navigate to requested angle.";
    }
    ROS_INFO("Finished service /rotate_robot");

    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_rotation");

  RobotRotate rotate;

  ros::spin();

  return 0;
}