#ifndef ROBOT_POSITION_AREA_H
#define ROBOT_POSITION_AREA_H

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <string>

class RobotPositionArea {
public:
  RobotPositionArea(ros::NodeHandle &nh, cv::Mat &frame);
  ~RobotPositionArea();

  // void update();
  void render();

private:
  ros::Subscriber sub_;
  nav_msgs::Odometry data_;
  std::string topic_name_;
  cv::Mat frame_;

  const std::string area_title_ = "Estimated robot position (Odometry):";
  const std::string window_titles_[3] = {"X", "Y", "Z"};
  const int cv_window_size_ = 80;
  const int cv_window_margin_ = 15;
  const double font_size_ = 0.5;
  const unsigned int font_color_ = 0xffffff;

  void msgCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif // ROBOT_POSITION_AREA_H