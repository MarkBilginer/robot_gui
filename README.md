# Description:
## This repository contains a user-friendly graphical interface developed using C++, ROS, and the CVUI library for interacting with mobile base robots remotely. The interface aims to simplify teleoperation tasks, allowing users with little or no previous experience in robotics to control the robots effectively.

## Key Features:
- Intuitive graphical interface for remote control of mobile base robots.
- Integration with ROS nodes for data communication and interaction with the robots.
- Designed to handle various robotic algorithms and provide a fallback option for manual teleoperation when algorithms fail.
- Customizable appearance and layout to suit different user preferences and requirements.

## Project Components:

### Graphical User Interface (GUI):
- Implements the visual interface using the CVUI library, presenting real-time data and control options to the user.
- **ROS Nodes**: Manages communication between the GUI and the robots, handling data transmission, sensor inputs, and robot control commands.
- **Teleoperation Node**: A custom ROS node developed as part of the project, facilitating manual teleoperation of the robots through the GUI.
- **Documentation**: Includes comprehensive documentation detailing the setup, usage, and architecture of the system, aiding in understanding and further development.
- **Overall**: This project aims to enhance the usability and accessibility of mobile base robots by providing an intuitive and user-friendly interface for remote interaction, contributing to the advancement of robotic research and experimentation.

### General Info Area:
- Display information published to the robot_info topic.

### Teleoperation Buttons:
- Provide buttons to control robot movement in the x-axis and rotation in the z-axis.

### Current Velocities:
- Show the current robot speed as "Linear Velocity" and "Angular Velocity" based on data from the cmd_vel topic.

### Robot Position (Odometry Based):
- Display the current position (x, y, z) of the robot based on data from the /odom topic.

### Distance Traveled Service:
- Provide a button to call the /get_distance service and display the response message.

## GUI Components:

### Initial State
![Initial State of Robot Control Panel](/images/robot_control_panel_0.png)
*The initial display of the Robot Control Panel presents a clean and organized interface awaiting data integration. It features fundamental robot control options—Forward, Backward, Left, Right, and Stop. Additionally, the interface includes placeholders for general information, which remain empty until relevant data is published. Key functional areas include a section for displaying the robot's current velocities, which update dynamically when the teleoperation buttons are pressed. The estimated robot position, based on odometry, updates autonomously. Lastly, the total distance traveled is updated through a manual trigger by pressing the 'Call' button.*

### Data Populated
![Data Populated in Robot Control Panel](/images/robot_control_panel_1.png)
*The General Info section now displays data about the robot including its description, serial number, IP address, and firmware version. This helps users verify the robot's identity and operational status.*

### Extended Information
![Extended Information Display](/images/robot_control_panel_2.png)
*Additional details are shown in the General Info section, including maximum payload and hydraulic oil parameters (temperature, tank fill level, and pressure), providing a comprehensive overview of the robot's current mechanical status.*

### Dynamic Updates
![Dynamic Updates on Robot Control Panel](/images/robot_control_panel_3.png)
*This view of the Robot Control Panel illustrates how the interface dynamically updates in response to user interactions. Pressing the teleoperation buttons results in real-time updates to both the current velocities (linear and angular) and the robot's estimated position based on odometry. The display reflects these changes immediately, showing the new X, Y, and Z coordinates to accurately track the robot's movement.*

### Distance Calculation
![Distance Traveled Calculation](/images/robot_control_panel_4.png)
*This view highlights the Distance Traveled feature of the Robot Control Panel. To update the total distance covered by the robot, the 'Call' button must be pressed. This action triggers a request to the distance_tracker_service, a separate ROS package available in this GitHub repository, which then calculates and displays the accumulated distance traveled as the robot moves. This integration showcases the modular design of the system, facilitating easy updates and maintenance.*

### Movement and Positioning
![Continued Movement and Positioning](/images/robot_control_panel_5.png)
*Further movement and interaction with the buttons updates the interface with new positional data and velocity changes, showcasing the GUI's ability to track detailed dynamics of robot movement.*

### Simulation View Start
![Robot in Simulation Environment](/images/robot_simulation_gazebo_0.png)
*Initial view of the robot in a simulated Gazebo environment, showcasing the GUI's integration with simulation tools for testing and development.*

### Simulation View with Movement
![Robot Movement in Simulation](/images/robot_simulation_gazebo_1.png)
*The robot navigates within the simulated environment, highlighting the real-time interaction capabilities of the GUI with simulated robotics scenarios.*


## Additional Dependencies

To ensure full functionality of the `robot_gui` project, it relies on several other ROS packages and GitHub repositories. These are essential for various features within the Robot Control Panel:

- **[robot_info](https://github.com/MarkBilginer/robot_info)**: This repository provides the essential robot information that populates the General Info area of the Robot Control Panel. It sources data dynamically and is crucial for displaying current robot status and details.

- **[distance_tracker_service](https://github.com/MarkBilginer/distance_tracker_service)**: This package includes a ROS service node that calculates the total distance traveled by the robot. It is invoked when the 'Call' button on the Robot Control Panel is pressed, making it integral for the Distance Traveled feature.

- **[robotinfo_msgs](https://github.com/MarkBilginer/robotinfo_msgs)**: This repository defines the `RobotInfo10Fields.msg`, which is utilized by the `robot_info` topic to structure the data sent to and displayed on the Robot Control Panel. This custom message format ensures that the interface can accurately and efficiently process and display the robot's data.

Each of these components is vital for the robot_gui to operate correctly, and they must be installed and configured according to their individual setup instructions.