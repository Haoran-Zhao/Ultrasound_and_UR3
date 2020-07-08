# Ultrasound_and_UR3

### This project is to use the new universal robot drive becasue of the deprecation of the ur_modern_driver

Install the new universal robot drive following the instruction in ['Universal Robot'](https://github.com/ros-industrial/universal_robot)

Install ['MoveIt'](https://moveit.ros.org/) 

## trouble shoot
1. ['Demo with fake controller works well, but real UR3e robot does not move'](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/106)

2. /follow_joint_trajectory issue: ['Not able to control the UR10e, issue with controllers'](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/55)(['see'](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/55#issuecomment-562215033))

3. ['Exception while loading move_group capability 'move_group/MoveGroupExecuteService' Melodic'](https://github.com/ros-industrial/universal_robot/issues/413)


#### 1. Connecting Universal Robot (UR3) to PC (Ubuntu 16.04)
##### 1.1 Configure your hardware
Follow the steps on the official website [`"Getting Started with a Universal Robot and ROS-Industrial"`](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) to set up network using a router.

To connect your PC and an UR3, run the following launch file, (you need to `source devel/setup.bash` first, and go to each folder that includes the launch file to launch it)
```
  roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.1.4 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml

  roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch

  roslaunch ur3_moveit_config moveit_rviz.launch config:=true
```
or run the launch file [`ur3_hardware/launch/rviz_config.launch`](https://github.com/Haoran-Zhao/US_UR3/blob/master/src/ur3_hardware/launch/rviz_config.launch)
You may need to change your joint limit file before you run any motion planning program on the hardware. ([`Example`](https://github.com/lihuang3/ur3_ROS-hardware/issues/1#issuecomment-422070509))

Get the joint states:
```
rostopic echo /joint_states
```
Get end effector position"
```
rosrun tf tf_echo /base_link /ee_link
```
#### 2. Network setup of Windows and Linux
Follow the instruction [`ROS/NetworkSetup`](http://wiki.ros.org/ROS/NetworkSetup)
##### 2.1 Linux troubleshooting
ssh Linux failed: port 22: Connection refused [`solution: reinstall shh`](https://stackoverflow.com/questions/17335728/connect-to-host-localhost-port-22-connection-refused)
##### 2.2 Windows troubleshooting
set ethernet as private, which is discoverable [`solution`](https://superuser.com/questions/627208/unable-to-ping-a-windows-machine-from-linux/1203485).

Allow ping through the firewall ICMPv4 private (do not enable domine!!!) [`solution`](https://www.faqforge.com/windows/windows-10/how-to-allow-ping-trough-the-firewall-in-windows-10/)

##### 2.3 ROS and Matlab network setting
ROS and Matlab network setting (can not recieve or publish topic)[`solution`](https://itectec.com/matlab/matlab-why-is-the-ros-subscriber-callback-in-matlab-not-triggered-when-messages-are-published-from-an-external-ros-node-not-in-matlab/)
in Linux change permanent
```
gedit .bashrc
```
if pc is not connected to ethernet, comment out last two line of .basrc file. 

#### 3. Jog Arm
Jog arm are from the repository [`Tokyo Opensource Robotics Kyokai Association
`](https://github.com/tork-a)
##### 3.1 Jog arm
The real time control code (Jog_ur3) are from the repository ['jog_arm'](https://github.com/UTNuclearRoboticsPublic/jog_arm)
General issues can be found in the [`issue`](https://github.com/UTNuclearRoboticsPublic/jog_arm/issues)

#### 4. Hardware Implementation
##### 4.1 jog_arm controller issues toubleshoot
1. [`jogging with real UR3 #93`](https://github.com/UTNuclearRoboticsPublic/jog_arm/issues/93)
2. [`ROS controller_manager`](http://wiki.ros.org/controller_manager)
3. [`Velocity streaming #218`](https://github.com/ros-industrial/ur_modern_driver/issues/218)
4. [`Adding JointGroupVelocityControllers #224`](https://github.com/rosindustrial/ur_modern_driver/pull/224/commits/e0032825cf1acaf5c81738f835eaf85410bdee84)

Can not jog arm with `joint_group_vel_controller`, but can make it work with following [`solution`](https://github.com/UTNuclearRoboticsPublic/jog_arm/issues/94#issuecomment-497584452), which pulish the joint velocity to `joint_speed` topic.

5. [`Connecting jog_arm to real UR3`](https://github.com/UTNuclearRoboticsPublic/jog_arm/issues/94)

##### 4.2 jog ur3 with Xbox 360 wired controller
![Screenshot from 2020-04-15 20-46-05](https://user-images.githubusercontent.com/16868368/79405908-44b9cc00-7f5b-11ea-88c4-2ea957eb76ac.png)

change the `<arg name="robot_ip" default="xxx.xxx.x.x"/>` in `src/jog_ur3/jog_ur3/launch/hardware_standard_init.launch`

```
cd ~/ws
source devel/setup.bash
catkin_make
roslaunch jog_ur3 ur3_hardware.launch
```

change linear speed in [`ur3_test.yaml`](https://github.com/Haoran-Zhao/US_UR3/blob/master/src/jog_ur3/jog_ur3/config/ur3_test.yaml)

##### 4.3 add tool and TCP on UR3
[`Creating a URDF with an UR5 robot and a custom end-effector`](https://gramaziokohler.github.io/compas_fab/latest/examples/03_backends_ros/07_ros_create_urdf_ur5_with_measurement_tool.html)
