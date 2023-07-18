
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

2. Turn on the Control device

3. access the desk app via {FCI-ip} while waiting for the robot to boot up

4. press "unlock joint" in the sidebar of Desk

5. Select [Activate FCI] from the drop-down menu in the top right corner of Desk GUI

6. Make sure the activation button is pulled up (or half-press the external enabling button to activate the robot)

7. Launch a terminal. Enter your desired workspace (i.e. catkin_ws). Source the setup file (i.e. source devel/setup.bash). 

8. In the terminal, enter ```roslaunch franka_example_controllers move_to_start.launch robot_ip:=172.16.0.2 load_gripper:=true robot:=panda```  to move the robot to the home position.

&nbsp;

## Franka ROS Controller Examples
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

<!--
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

After cloning the repo, you need to first follow the following steps in order to use the interface.

1. ```cp src/franka_ros_interface/franka.sh ./```
2. Configure the franka.sh file (enter {FCI-ip} in FRANKA_ROBOT_IP and your current ip in your_ip, and also the ros_version)

### Usage
1. Run ```./franka.sh master``` in the terminal
2. Run ```roslaunch franka_interface interface.launch start_moveit:=false``` -> starts driver node

3. Open another terminal and run  ```./franka.sh master```  again.
4. Run ```rosrun franka_interface demo_joint_positions_keyboard.py``` -->


# Voice Control Panda with Alexa Skill

## Installations:
1. Installing Flask-ask 
```
pip3 install https://github.com/johnwheeler/flask-ask/master.zip
pip install -r requirements-dev.txt
```
Import failure might arise due to the package versions are conflicting. This problem can be resolved with the following steps: 
```
pip uninstall jinja2
pip install jinja2==3.0
pip uninstall itsdangerous
pip install itsdangerous==2.0.1
```

2. Since Alexa skills should run either behind a public HTTPS server or a Lambda function, we'll use an open source command-line program called ngrok (http://ngrok.com/) for this purpose as it helps in opening a secure tunnel to the localhost and exposes it behind the HTTPS endpoint. 
create an account and follow the setup and installation steps below:
    
    a. download the binary zip file

    b. unzip it using ```unzip /path/to/ngrok.zip```

    c. add your authtoken to the default ngrok.yml configuration file
    ```ngrok config add-authtoken 2PQRT1B4jUlqQPYK1Q0QZgKH6cL_4iDn1KPQha57UHif7hvqx```
    
    d. To start a HTTP tunnel forwarding to your local port 5000, run the following command: ```./ngrok http 5000```

<br/>

## Operating Procedure:
1. Setup the panda robot and activate FCI mode
2. Open a termianl and type in ```./ngrok http 5000```
3. Copy the HTTPS endpoint link at the end in the ngrok terminal and paste it in the alexa skill's endpoint settings
4. Open another terminal and go to the workspace
5. Build the packages and source it
```
    catkin build
    source devel/setup.bash
```
6. Launch the pnp_server for the panda robot and the alexa skill server  

```
    roslaunch panda_pnp_srvcli pick_place.launch voice:=true
```


Or if you are trying to use simulations for the robot, then launch the following file instead:
```
    roslaunch panda_pnp_srvcli pick_place.launch voice:=true sim:=true
```

7. In the "test" tab of the alexa skill, type in or say the following phrases:

    a. ```alexa launch voice control panda.```

    b. ```alexa give me the {pipe_type} pipe.``` The {pipe_type} in the command can be short or long depending on which pipe you want it to pick up.

    c. or simply just combine them: ```alexa launch voice control panda and give me the {pipe_type} pipe.```

    Or if you are using an alexa echo dot, just say the commands above.

8. Watch the robot picking up the pipe for you

&nbsp;


## Control Panda with rostopics

If you don't want to use voice control, you can also use ros client to send request to pick up certain type of pipes.

1. Launch the pnp_server for the panda robot
```
    roslaunch panda_pnp_srvcli pick_place.launch 
```

2. Open another terminal and source the path. Then run the following command:
```
    rosrun panda_pnp_srvcli pnp_client {pipe_type}
```

## Resolving package issues

When using catkin build to build the packages, there might be an error indicating that the cv-bridge package is missing. This might be caused by the installing the ffmpeg package required by whisper, which accidently removed the opencv package, resulting in this issue. To resolve this, please follow the steps:

1. sudo aptitude install libopencv-dev
2. hit "n" twice and then hit "Y" twice to remove the unnecessary packages
3. sudo aptitude install libopencv-dev
4. hit "n" twice and then hit "Y" twice to install opencv
5. sudo apt install ros-noetic-cv-bridge

After this, you should be able to build the packages without error. However, the problem is now you have to reinstall the ffmpeg package for whisper when you want to use it. Use the following command to install it. Once it is installed, you'll need to follow the steps above again if you want to use catkin build to build the packages.

1. sudo apt-get -qq install -y ffmpeg 


 
