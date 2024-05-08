#include "robot_gui/robot_position_area.h"
#include "robot_gui/cvui.h"

RobotPositionArea::RobotPositionArea(ros::NodeHandle &nh, cv::Mat &frame)
    : frame_(frame) {
  topic_name_ = "odom";
  sub_ = nh.subscribe<nav_msgs::Odometry>(
      topic_name_, 2, &RobotPositionArea::msgCallback, this);
};

RobotPositionArea::~RobotPositionArea(){};

void RobotPositionArea::msgCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  data_ = *msg;
};

void RobotPositionArea::render() {

  const int window_x_position_x = cv_window_margin_;
  const int window_y_position_x = cv_window_margin_ * 2 + cv_window_size_;
  const int window_z_position_x = cv_window_margin_ * 3 + cv_window_size_ * 2;

  const int window_x_text_position_x = cv_window_margin_ + cv_window_size_ / 3;
  const int window_y_text_position_x =
      cv_window_margin_ * 2 + cv_window_size_ / 3 + cv_window_size_;
  const int window_z_text_position_x =
      cv_window_margin_ * 3 + cv_window_size_ / 3 + cv_window_size_ * 2;

  const int windows_position_y = 520;
  const int windows_text_position_y =
      windows_position_y + 3 * cv_window_size_ / 4;

  // Area Title
  cvui::window(frame_, window_x_position_x, windows_position_y - 20, 270, 15,
               area_title_);

  // Robot Position x
  cvui::window(frame_, window_x_position_x, windows_position_y, cv_window_size_,
               cv_window_size_, window_titles_[0]);
  cvui::printf(frame_, window_x_text_position_x, windows_text_position_y,
               font_size_, font_color_, "%0.1f", data_.pose.pose.position.x);

  // Robot Position y
  cvui::window(frame_, window_y_position_x, windows_position_y, cv_window_size_,
               cv_window_size_, window_titles_[1]);
  cvui::printf(frame_, window_y_text_position_x, windows_text_position_y,
               font_size_, font_color_, "%0.1f", data_.pose.pose.position.y);

  // Robot Position z
  cvui::window(frame_, window_z_position_x, windows_position_y, cv_window_size_,
               cv_window_size_, window_titles_[2]);
  cvui::printf(frame_, window_z_text_position_x, windows_text_position_y,
               font_size_, font_color_, "%0.1f", data_.pose.pose.position.z);
};