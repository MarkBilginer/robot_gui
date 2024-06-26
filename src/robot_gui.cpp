#include "robot_gui/robot_gui.h"
//#include "robot_gui/distance_tracker_area.h"
//#include "robot_gui/robot_position_area.h"

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

constexpr char WINDOW_NAME[] = "Robot Control Panel";

RobotGui::RobotGui(ros::NodeHandle &nh)
    : nh_(nh), frame(cv::Mat(800, 300, CV_8UC3, cv::Scalar(49, 52, 49))),
      generalInfoArea_(nh_, frame), teleoperationButtonsArea_(nh_, frame),
      robotPositionArea_(nh_, frame), distanceTrackerArea_(nh_, frame) {
  cvui::init(WINDOW_NAME);
}

RobotGui::~RobotGui() {
  // Destructor implementation
}

void RobotGui::run() {
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    render();
    teleoperationButtonsArea_.publishTwist();
    int key = cv::waitKey(10);

    if (key == 27) {
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
}

void RobotGui::render() {
  // Call render methods of each component
  // generalInfoArea_.update();

  generalInfoArea_.render();
  teleoperationButtonsArea_.render();
  robotPositionArea_.render();
  distanceTrackerArea_.render();
  // Similarly for other components
  cvui::update();
  cvui::imshow(WINDOW_NAME, frame);
}