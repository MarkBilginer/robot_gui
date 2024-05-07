#include "robot_gui/robot_gui.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  ros::NodeHandle nh;

  RobotGui robot_gui(nh);
  robot_gui.run();

  return 0;
}