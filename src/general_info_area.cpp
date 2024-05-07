#include "robot_gui/general_info_area.h"
#include "robot_gui/cvui.h"

GeneralInfoArea::GeneralInfoArea(ros::NodeHandle &nh, cv::Mat &frame)
    : frame_(frame) {
  // Constructor implementation
  topic_name_ = "robot_info"; // Topic from which to receive general info
  sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      topic_name_, 2, &GeneralInfoArea::msgCallback, this);
}

GeneralInfoArea::~GeneralInfoArea() {
  // Cleanup code here, if any
}

void GeneralInfoArea::msgCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  // Clear previous data
  info_lines_.clear();
  // ROS_INFO("Callback triggered");
  // ROS_INFO("Received data_field_01: %s", msg->data_field_01.c_str());

  // Assuming RobotInfo10Fields message has exactly 10 string fields named
  // data_field_01 to data_field_10
  info_lines_.push_back(msg->data_field_01);
  info_lines_.push_back(msg->data_field_02);
  info_lines_.push_back(msg->data_field_03);
  info_lines_.push_back(msg->data_field_04);
  info_lines_.push_back(msg->data_field_05);
  info_lines_.push_back(msg->data_field_06);
  info_lines_.push_back(msg->data_field_07);
  info_lines_.push_back(msg->data_field_08);
  info_lines_.push_back(msg->data_field_09);
  info_lines_.push_back(msg->data_field_10);

  // ROS_DEBUG_STREAM("Updated robot info: " << msg->data_field_01);
}

// void GeneralInfoArea::update() {
//   // Update logic for the general info area
// }

void GeneralInfoArea::render() {
  // Rendering logic for the general info area
  // Setup GUI elements
  frame_ = cv::Scalar(39, 42, 39); // Background color
  const int theX = 25, theY = 20, theWidth = 250;
  int lineHeight = 15;
  int theHeight =
      std::max(100, 20 + static_cast<int>(info_lines_.size()) * lineHeight);
  const cv::String theTitle = "General Info";
  cvui::window(frame_, theX, theY, theWidth, theHeight, theTitle);

  // Display each line in the vector
  int startY = 45; // Start Y position for text
  for (const auto &line : info_lines_) {
    cvui::printf(frame_, 45, startY, 0.4, 0xFFFFFF, line.c_str());
    startY += lineHeight; // Move down for the next line
  }

  cvui::update();
}
