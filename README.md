# allassignmens-35
This Github Repository is part of the Autonomous Vehicles and Artificial Intelligence Course WS 22/23. <br />

## About
Autonomous driving is the future of individual mobility and all major manufacturers are working on fully autonomous vehicles. While there are robust and good solutions for the individual problems in autonomous driving, the main challenge lies in their integration. Altogether, an autonomous vehicle’s software is the biggest problem. Therefore, the key in self-driving vehicles is about getting the software right. In this course, we will investigate the different aspects of self-driving vehicles as well as the importance and application of artificial intelligence in this domain from a theoretical but especially from a practical perspective. 

## Basic Setup
1. Clone this repository.
2. Install all libraries from the correct requirements.txt.
3. Go to /allassignments-35/src/ and run ```colcon build``` to compile all ros2 nodes.

To start the program you need to first start the turtlebot using its bringup. Next, for stable odometry, launch the ros2 cartographer using ```ros2 launch turtlebot3_cartographer cartographer.launch.py```. If the cartographer shows errors, make sure you are connected to the same network as the turtlebot or try to restart the turtlebot. 
Now start all required ros2 nodes:
For each node source first with ```source install/local_setup.bash```. Then run:
```
ros2 run camera_pkg turtlebotslam_node
ros2 run camera_pkg camera_node
ros2 run camera_pkg image_processing_node
ros2 run lidar_pkg lidar

ros2 run camera_pkg gui_node

ros2 run camera_pkg occupancy_map_node
ros2 run movement movement_controller
```
You need to input a camera frequency on the GUI, so that the driving system starts executing.
To reset the driving system you only need to restart the occupancy map and the movement controller.

### Controls
- Press P to terminate the movement controller to stop sending movement commands.
- Use W/A/S/D to drive the turtlebot manually using the remote controller. Use E/R to increase/decrease the speed to the turtlebot. The remote controller must be started separately using ```ros2 run remote_control cont```.

## Node architecture
![Kopie von UML_Diagram drawio](https://user-images.githubusercontent.com/65449566/222164328-5308e7b1-5067-46c1-a3b1-f130556a12a8.png)

## Demo
### Simple racetrack without curves 

  [![Simple racetrack without curves](https://img.youtube.com/vi/JIVdeuSpd4E/0.jpg)](https://youtu.be/JIVdeuSpd4E "Simple racetrack without curves")

### Circle racetrack

  [![Round racetrack](https://img.youtube.com/vi/aXM5ywss980/0.jpg)](https://youtu.be/aXM5ywss980 "Round racetrack")

