#ifndef DISTANCE_TRACKER_AREA_H
#define DISTANCE_TRACKER_AREA_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <string>

class DistanceTrackerArea {
public:
  DistanceTrackerArea(ros::NodeHandle &nh, cv::Mat &frame);
  ~DistanceTrackerArea();

  // void update();
  void render();

private:
  ros::ServiceClient distance_client_;
  std::string distance_message_;
  std::string topic_name_;
  cv::Mat frame_;

  const std::string area_title_ = "Distance travelled:";
  const std::string window_title_ = "Distance in meters (m):";
  const int cv_button_size_ = 80;
  const int margin_ = 15;
  const double font_size_ = 0.5;
  const unsigned int font_color_ = 0xffffff;

  void callGetDistanceService();
};

#endif // DISTANCE_TRACKER_AREA_H