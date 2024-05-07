#ifndef TELEOPERATION_BUTTONS_AREA_H
#define TELEOPERATION_BUTTONS_AREA_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <string>

class TeleOperationButtonsArea {
public:
  TeleOperationButtonsArea(ros::NodeHandle &nh, cv::Mat &frame);
  ~TeleOperationButtonsArea();

  // void update();
  void render();
  void publishTwist();

private:
  ros::Publisher twist_pub_;
  // Create Twist message
  geometry_msgs::Twist twist_msg_;
  std::string twist_topic_name_;
  float linear_velocity_step_ = 0.1;
  float angular_velocity_step_ = 0.1;
  cv::Mat frame_;

  const int buttonWidth_ = 80;
  const int buttonHeight_ = 50;
  const int baseYMargin_ = 215;
  const int buttonYMargin_ = 20;
  const int buttonXMargin_ = 15;
  const int textMargin_ = 5;

  void displayVelocities();
};

#endif //  TELEOPERATION_BUTTONS_AREA_H