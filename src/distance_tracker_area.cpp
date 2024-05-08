#include "robot_gui/distance_tracker_area.h"
#include "robot_gui/cvui.h"
#include "std_srvs/Trigger.h"
#include <future>

DistanceTrackerArea::DistanceTrackerArea(ros::NodeHandle &nh, cv::Mat &frame)
    : frame_(frame) {
  topic_name_ = "get_distance";
  distance_client_ = nh.serviceClient<std_srvs::Trigger>(topic_name_);
};

DistanceTrackerArea::~DistanceTrackerArea(){};

void DistanceTrackerArea::callGetDistanceService() {

  std_srvs::Trigger srv_req;

  if (distance_client_.call(srv_req)) {
    ROS_INFO("Response message: %s", srv_req.response.message.c_str());
    distance_message_ = srv_req.response.message;
    // Display this message somewhere on your GUI
    // You might want to store this in a member variable to display in render()
  } else {
    ROS_INFO("Failed to call service get_distance");
    distance_message_ = "Error";
  }
}

void DistanceTrackerArea::render() {

  const int areaStartY = 615;
  const int areaWidth = 300 - 2 * margin_;
  const int areaTitleHeight = 15;
  const int buttonStartY = areaStartY + areaTitleHeight + 5;
  const int distanceDisplayX = margin_ * 2 + cv_button_size_;
  const int distanceDisplayWidth = areaWidth - cv_button_size_ - margin_;
  const int distanceDisplayHeight = cv_button_size_;

  // Area Title
  cvui::window(frame_, margin_, areaStartY, areaWidth, areaTitleHeight,
               area_title_);

  // Get distance button
  if (cvui::button(frame_, margin_, buttonStartY, cv_button_size_,
                   cv_button_size_, "Call")) {

    callGetDistanceService();
  }

  cvui::window(frame_, distanceDisplayX, buttonStartY, distanceDisplayWidth,
               distanceDisplayHeight, window_title_);

  cvui::printf(frame_, distanceDisplayX + distanceDisplayWidth / 2,
               buttonStartY + 3 * distanceDisplayHeight / 4, font_size_,
               font_color_, "%s", distance_message_.c_str());
};