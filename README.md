# Project Description
This ros2 project aims to use a camera to allow a robot to follow a yellow ball in a gazebo environment. This will be done by creating a ros2 node in python to implement this functionality.

## Functionality
The ros2 node has many functionalities:
1. The node subscribes to the robot's camera ('/camera/image_raw') to be able recieve the live feed that the robot and observing. 
2. The node publishes Twist messages to the 'cmd_vel' topic so that the robot can move accurately based on what the camera is observing
3. The node will use CvBridge to convert the images recieved from the ros2 topics into opencv format and vice versa
4. The robot will keep rotating in its place until it finds a yellow ball
5. After finding the ball it will adjust its orientation and then move towards it 

## Usage
- Clone the repository:
```bash
git clone git@github.com:Mohamad-Nasser-engr/Robotics-Session13-Assignment.git
```
- Build the workspace:
```bash
cd Robotics-Session13-Assignment/Ball_follower
```
```bash
colcon build
```
- Source ROS2 and bashrc:
```bash
source /opt/ros/humble/setup.bash
```
```bash
source install/setup.bash
```
- Open a gazebo environment and add a yellow ball in it:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py 
```
Note: The yellow ball created during testing had a radius of 0.2 m, and color parameters as follows, r:1.0, g:1.0, b:0.0, a:1.0
- Launch the node:
```bash
ros2 launch launch/ball_tracker_launcher.py
```

