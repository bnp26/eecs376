
This package is intended for governing a 2D turtlebot or other cylindrical robots. It contains several things:

	1) eecs376_ps3_path_services
		This is a library that contains all functionality necessary to consume a list
		of waypoints and produce a list of (theta, distance) pairs of commands the robot should
		perform to reach each waypoint.

	2) eecs376_ps3_path_services_node
		An executable that provides a service called "path_service" which takes a PathSrv object (see "srv" folder).
		It produces geometry_msgs/Twist commands on the topic "/robot0/cmd_vel"

	3) eecs376)ps3_path_client_node
		An executable that is responsible for producing waypoints. It is a client to the "path_service" service
		and publishes a list of waypoints (nav_msgs/Path) within a PathSrv object.
