#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include "robot_gui/general_info_area.h"
#include "robot_gui/teleoperation_buttons_area.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>

class RobotGui {
public:
  RobotGui(ros::NodeHandle &nh);
  ~RobotGui();

  // void init();
  void run();

private:
  ros::NodeHandle &nh_;
  cv::Mat frame;
  GeneralInfoArea generalInfoArea_;
  TeleOperationButtonsArea teleoperationButtonsArea_;

  void render();
};

#endif // ROBOT_GUI_H