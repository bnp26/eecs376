
Mobile Robotics Homework 2...making a bad roomba.

The goals of this homework are to control a robot using the stdr_simulator ROS package that is equipped with a lidar.
The idea is to have the robot spin in a (pseudo-random) direction if the robot detects that it will run into an obstacle.

This package contains two nodes and a library, and also comes with unit tests:
	eecs376_ps2_lidar_alarm 			(library)
	eecs376_ps2_lidar_alarm_node 		(executable)
	eecs376_ps2_reactive_commander_node (executable)
	eecs476_ps2_lidar_alarm-test		(executable -- unit test)

To test, use the command:
	catkin_make run_tests
