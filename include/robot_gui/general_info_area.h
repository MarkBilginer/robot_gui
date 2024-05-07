#ifndef GENERAL_INFO_AREA_H
#define GENERAL_INFO_AREA_H

#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class GeneralInfoArea {
public:
  GeneralInfoArea(ros::NodeHandle &nh, cv::Mat &frame);
  ~GeneralInfoArea();

  // void update();
  void render();

private:
  ros::Subscriber sub_;
  std::vector<std::string> info_lines_;
  std::string topic_name_;
  cv::Mat &frame_;
  bool info_changed_ = true;

  void msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
};

#endif // GENERAL_INFO_AREA_H