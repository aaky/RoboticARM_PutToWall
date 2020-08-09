# Put To Wall simulation and testing
This repository consist of multiple packages which help to test workspace analysis of Put to Wall sortation system simulation using Robotic arm Aubo i5L/i10.

## Building 

	mkdir catkin_ws
	cd catkin_ws
	mkdir src
	cd src
	git clone https://github.com/codeGreensam/PUTtoWALL.git
	cd ..
	catkin build

Aboce instructions should build all the packages. ROS-Kinetic, Moveit , TracIK , Gazebo installation are pre-requisite of running packages in this repo.

## Running simulation

Spawn up robot and move group in Rviz

	roslaunch put_to_wall_moveit_config demo.launch

Generate Planning scene which includes collision obstacle of Octagonal shape

	rosrun put_to_wall_manipulation generate_planning_scene

Start simulation for picking and dropping virtual object in all the drop bins 

	rosrun put_to_wall_manipulation put_to_wall_sim
	
