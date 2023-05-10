
# Franka Panda Setup Guide
This guide describes how to setup the Franka Panda and how to use the Desk interface as well as ROS to control it. 

## Installation for libfranka and franka_ros
libfranka is the open source C++ interface for the panda robot, and franka_ros connects it with the entire ROS ecosystem.
Follow the official guide to finish setting up the linux PC and install the latest version of libfranka and franka_ros.
[official panda setup documentation](https://frankaemika.github.io/docs/installation_linux.html)


## Use Desk to operate Panda
There are two ways to connect the robot to the workstation PC, the first and most simple way to do this is to connect the PC and the robot base using an Ethernet cable. The downside for this is that by choosing this option, you can only operate the robot using Desk, and you won't be able to use ROS or other interfaces to control it.

Another option is connecting the PC and the Control box using an Ethernet cable. But one thing to note is that using the second option, you have to change the networks settings in order to make it work.

The following is the ip address for the robot case and the Control box.
* {robot-ip}: 192.168.1.1
* {FCI-ip}: 172.16.0.2

### steps:
1. switch on Control with the external activation device pressed down -> flash yellow

2. wait for the robot to boot up -> yellow light

3. access Desk through the browser at https://{robot-ip} or https://{FCI-ip}  depending on which port you are connecting to (robot base or control box).

4. login to the robot with the correct credentials or complete the setup guide if it is the first time activating the robot. 

Now you should be able to see the Franka Desk GUI.
5. press "unlock joint" in the sidebar of Desk -> white light

6. press enabling button and guiding button at the same time [teaching mode] -> white light

7. take the external enabling device and possibly also the emergency stop device out
of the Hazardous Zone. 

8. half-press the external enabling button [robot activated]-> blue light

9. press the Play button in Desk -> green light

* in case of error -> red light

* conflicting authorization signal -> pink light 

&nbsp;

## Use franka_ros to operate Panda


1. Connect the Control device and PC using the Ethernet cable and configure the network setting (change IPv4 from DHCP to manual and enter 172.16.0.1 for address and 24 for Netmask)

2. switch on the Control device

3. access the desk app via {FCI-ip} while waiting for the robot to boot up

4. press "unlock joint" in the sidebar of Desk

5. Select [Activate FCI] from the drop-down menu in the top right corner of Desk GUI

6. half-press the external enabling button to activate the robot

7. Launch a terminal and enter ```roslaunch franka_example_controllers move_to_start.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda```  before using any other ROS controllers to control the robot (otherwise it will generate an error, since all the example starts from this starting position).

&nbsp;

## ROS Controller Examples
* package descriptions and controller details can be found at [franka_example_controllers_plugin.xml]
1. ```roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ``` -> use a interactive interface in RViz to control the cartesian rotation and transformation of the robot end effector.

2. ```roslaunch franka_example_controllers cartesian_pose_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ```

3. ```roslaunch franka_example_controllers cartesian_velocity_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ``` 

4. ```roslaunch franka_example_controllers elbow_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ``` -> fixing EE and move elbow 

5. ```roslaunch franka_example_controllers force_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ```

6. ```roslaunch franka_example_controllers joint_impedance_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ```

7. ```roslaunch franka_example_controllers joint_position_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ```

8. ```roslaunch franka_example_controllers joint_velocity_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ```

9. ```roslaunch franka_example_controllers model_example_controller.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda ``` -> evaluates and prints the dynamic model of Franka

&nbsp;

## MoveIt Example

1. ``` roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true``` -> can use MoveIt to plan and execute robot motion

&nbsp;

## Franka_ros_interface
Reference: <https://github.com/justagist/franka_ros_interface>

A ROS interface library for the Franka Emika Panda robot (real and simulated), extending the franka-ros library to expose more information about the robot, and providing low-level control of the robot using ROS and Python API.

### Installation
```
cd <catkin_ws>
git clone -b v0.7.1 https://github.com/justagist/franka_ros_interface src/franka_ros_interface
catkin build franka_ros_interface # or catkin_make
source devel/setup.bash
```

After installing cloning the repo, you need to first follow the following steps in order to use the interface.

1. ```cp src/franka_ros_interface/franka.sh ./```
2. Configure the franka.sh file (enter {FCI-ip} in FRANKA_ROBOT_IP and your current ip in your_ip, and also the ros_version)

### Usage
1. Run ```./franka.sh master``` in the terminal
2. Run ```roslaunch franka_interface interface.launch start_moveit:=false``` -> starts driver node

3. Open another terminal and run  ```./franka.sh master```  again.
4. Run ```rosrun franka_interface demo_joint_positions_keyboard.py```



 