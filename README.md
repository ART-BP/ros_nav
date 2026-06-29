#  Omni-directional Mobie robot
## Requirement

ubuntu20.04

ros noetic

gazebo11(安装时注意区分用的是arm64的源还是x86的源)

一系列ros navigation相关的包，编译时会报出缺少的

## Create Workspace

```
$ mkdir -p catkin_ws/src
$ unzip ~/omni_robot.zip -d ~/catkin_ws/src
$ cd catkin_ws
$ catkin_make
编译会失败很多次，每次会报出缺少的包
```
## Demo on  RViz(这部分没什么用，可以跳过)
### Move to Pose
First, run omni fake
```
roslaunch omni_fake omni_fake.launch
```
Then, run the controller
```
$ rosrun omni_control move2pose
```
On Rviz simulator, select ```2D Nav Goal``` on the toolbar and select goal position for robot. <br>
Results: <br>
![Move 2 Pose](omni_control/results/move2pose.png)

### Pure Pursuit
First, run omni fake and path generator
```
$ roslaunch omni_fake omni_fake.launch
$ roslaunch omni_path_generator simple_path.launch
```
Then, run the controller
```
$ rosrun omni_control pure_pursuit
```
![Pure Pursuit](omni_control/results/pure_pursuit.png)

## Navigation Simulation on Gazebo
### Build map with Gmapping
Using keyboard to control the robot to explore the room and build the map
```
roslaunch omni_gazebo gazebo.launch
roslaunch omni_gmapping slam_gmapping.launch
rosrun omni_teleop teleop
```
When the map is complete, run map_saver to save the map
```
rosrun map_server map_saver [-f mapname]
```


### Run ros_navigation 
```
roslaunch omni_gazebo gazebo.launch
roslaunch omni_navigation omni_navigation.launch  local_planner:=teb
```
Candidate local_planner: `teb`, `dwa`, `neupan`

For NeuPAN, activate the Python environment that provides the `neupan` package
before launching:

```
source ~/anaconda3/etc/profile.d/conda.sh
conda activate neupan38
source /opt/ros/noetic/setup.bash
source <catkin_ws>/devel/setup.bash
roslaunch omni_navigation sim_navigation.launch local_planner:=neupan
```

## Topic
* /cmd_vel <```geometry_msgs/Twist```> - The robot will get the value from this topic to control the robot velocity.
* /odom <```navigation_msgs/Odometry```> - The robot state include real pose and velocity of robot
* /scan <```sensor_msgs/LaserScan```> - The lidar 2D topic which show the scan value of lidar
* /camera/rgb/image_raw <```sensor_msgs/Image```> - The rbg topic which show the depth image stream (8UC3) of rgbd camera
* /camera/depth/image_raw <```sensor_msgs/Image```> - The depth topic which show the depth image stream (32FC1) of rgbd camera
