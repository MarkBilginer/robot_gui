#include "robot_gui/teleoperation_buttons_area.h"
#include "robot_gui/cvui.h"

TeleOperationButtonsArea::TeleOperationButtonsArea(ros::NodeHandle &nh,
                                                   cv::Mat &frame)
    : frame_(frame) {
  twist_topic_name_ = "cmd_vel";
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name_, 10);

  // Initialize twist message with zero velocities
  twist_msg_.linear.x = 0.0;
  twist_msg_.linear.y = 0.0;
  twist_msg_.linear.z = 0.0;
  twist_msg_.angular.x = 0.0;
  twist_msg_.angular.y = 0.0;
  twist_msg_.angular.z = 0.0;
}

TeleOperationButtonsArea::~TeleOperationButtonsArea() {}

// void TeleOperationButtonsArea::update() {}

void TeleOperationButtonsArea::publishTwist() {
  twist_pub_.publish(twist_msg_);
}

void TeleOperationButtonsArea::displayVelocities() {
  const int windowY = baseYMargin_ + buttonYMargin_ * 4 + buttonHeight_ * 3;
  const int textY =
      baseYMargin_ + buttonYMargin_ * 5 + buttonHeight_ * 3 + textMargin_;

  cvui::window(frame_, 15, windowY, 120, 40, "Linear velocity:");
  cvui::printf(frame_, 30, textY, 0.4, 0xff0000, "%.02f m/sec",
               twist_msg_.linear.x);

  cvui::window(frame_, 150, windowY, 120, 40, "Angular velocity:");
  cvui::printf(frame_, 165, textY, 0.4, 0xff0000, "%.02f rad/sec",
               twist_msg_.angular.z);
}

void TeleOperationButtonsArea::render() {
  // Render the buttons and handle their interaction
  const int forwardButtonX = buttonXMargin_ * 2 + buttonWidth_;
  const int stopButtonX = forwardButtonX;
  const int backwardButtonX = stopButtonX;
  const int leftButtonX = buttonXMargin_;
  const int rightButtonX = buttonXMargin_ * 3 + buttonWidth_ * 2;

  const int forwardButtonY = baseYMargin_ + buttonYMargin_;
  const int stopButtonY = baseYMargin_ + buttonYMargin_ * 2 + buttonHeight_;
  const int backwardButtonY =
      baseYMargin_ + buttonYMargin_ * 3 + buttonHeight_ * 2;
  const int leftButtonY = stopButtonY;
  const int rightButtonY = leftButtonY;

  if (cvui::button(frame_, forwardButtonX, forwardButtonY, buttonWidth_,
                   buttonHeight_, "Forward")) {
    twist_msg_.linear.x += linear_velocity_step_;
  }

  if (cvui::button(frame_, stopButtonX, stopButtonY, buttonWidth_,
                   buttonHeight_, "Stop")) {
    twist_msg_.linear.x = 0.0;
    twist_msg_.angular.z = 0.0;
  }

  if (cvui::button(frame_, leftButtonX, leftButtonY, buttonWidth_,
                   buttonHeight_, "Left")) {
    twist_msg_.angular.z += angular_velocity_step_;
  }

  if (cvui::button(frame_, rightButtonX, rightButtonY, buttonWidth_,
                   buttonHeight_, "Right")) {
    twist_msg_.angular.z -= angular_velocity_step_;
  }
  if (cvui::button(frame_, backwardButtonX, backwardButtonY, buttonWidth_,
                   buttonHeight_, "Backward")) {
    twist_msg_.linear.x -= linear_velocity_step_;
  }
  // Display the current velocities
  displayVelocities();
  cvui::update();
}