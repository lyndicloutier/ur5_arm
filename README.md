# UR5 Arm Manipulation

## Description
* Launch a simulation of the UR5 arm in Gazebo using MoveIt (ROS2 Humble)

## How to run the program
* To use the UR5 simulation, launch with MoveIt / Gazebo using the terminal: ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
* Run the server node first: ros2 launch ur5_arm_pkg ur5_arm_test.launch.py
* And then launch the client node afterwards: ros2 run ur5_arm_pkg service_test
