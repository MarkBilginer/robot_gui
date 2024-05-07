Description:
This repository contains a user-friendly graphical interface developed using C++, ROS, and the CVUI library for interacting with mobile base robots remotely. The interface aims to simplify teleoperation tasks, allowing users with little or no previous experience in robotics to control the robots effectively.

Key Features:

Intuitive graphical interface for remote control of mobile base robots.
Integration with ROS nodes for data communication and interaction with the robots.
Designed to handle various robotic algorithms and provide a fallback option for manual teleoperation when algorithms fail.
Customizable appearance and layout to suit different user preferences and requirements.
Project Components:

Graphical User Interface (GUI): Implements the visual interface using the CVUI library, presenting real-time data and control options to the user.
ROS Nodes: Manages communication between the GUI and the robots, handling data transmission, sensor inputs, and robot control commands.
Teleoperation Node: A custom ROS node developed as part of the project, facilitating manual teleoperation of the robots through the GUI.
Documentation: Includes comprehensive documentation detailing the setup, usage, and architecture of the system, aiding in understanding and further development.
Overall, this project aims to enhance the usability and accessibility of mobile base robots by providing an intuitive and user-friendly interface for remote interaction, contributing to the advancement of robotic research and experimentation.

General Info Area:
Display information published to the robot_info topic.

Teleoperation Buttons:
Provide buttons to control robot movement in the x-axis and rotation in the z-axis.

Current Velocities:
Show the current robot speed as "Linear Velocity" and "Angular Velocity" based on data from the cmd_vel topic.

Robot Position (Odometry Based):
Display the current position (x, y, z) of the robot based on data from the /odom topic.

Distance Traveled Service:
Provide a button to call the /get_distance service and display the response message.