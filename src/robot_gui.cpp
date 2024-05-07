#include "robot_gui/robot_gui.h"

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

constexpr char WINDOW_NAME[] = "Robot Control Panel";

RobotGui::RobotGui(ros::NodeHandle &nh)
    : nh_(nh), frame(cv::Mat(800, 300, CV_8UC3, cv::Scalar(49, 52, 49))),
      generalInfoArea_(nh_, frame) {
  cvui::init(WINDOW_NAME);
}

RobotGui::~RobotGui() {
  // Destructor implementation
}

void RobotGui::run() {
  while (true) {
    render();
    cvui::imshow(WINDOW_NAME, frame);
    int key = cv::waitKey(20);

    if (key == 27) {
      break;
    }

    ros::spinOnce();
  }
  ros::shutdown();
}

void RobotGui::render() {
  // Call render methods of each component
  // generalInfoArea_.update();
  generalInfoArea_.render();

  // Similarly for other components
}