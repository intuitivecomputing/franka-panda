
/*
This C++ program uses the MoveIt! library to simulate the Franka gripper performing a pick-and-place action.
The purpose of this executable is to verify the trajectory and the pose of the collision objects.

      It can be launched by the following command:
        roslaunch panda_pick_and_place pick_place_sim.launch 
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include "panda_pnp_srvcli/PnpRequest.h"

// read predefined poses in json
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <ros/package.h>

double deg_to_rad(double deg){
  return deg/180 * M_PI;
}

// The openGripper function sets joint values for the Franka gripper's finger joints to open the gripper, plans a trajectory for the gripper using MoveIt!, and executes the trajectory. 
void openGripper(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan plan_gripper)
{
   std::map<std::string, double> joints;
   joints = {{"panda_finger_joint1", 0.03},
              {"panda_finger_joint2", 0.03}};

    move_group.setJointValueTarget(joints);
    bool success = (move_group.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

}

// The closedGripper function does the same as the openGripper function but sets the joint values to close the gripper.
void closedGripper(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan plan_gripper)
{
   std::map<std::string, double> joints;
   joints = {{"panda_finger_joint1", 0.011},
              {"panda_finger_joint2", 0.011}};

    move_group.setJointValueTarget(joints);
    bool success = (move_group.plan(plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();

}

// The goHome function sets joint values for the robotic arm to move it to a pre-defined home position, plans a trajectory for the arm using MoveIt!, and executes the trajectory.
void goHome(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan plan_arm)
{
   std::map<std::string, double> home;
   home = {{"panda_joint1", 0},
            {"panda_joint2", -0.785398163397},
            {"panda_joint3", 0},
            {"panda_joint4", -2.35619449019},
            {"panda_joint5", 0},
            {"panda_joint6", 1.57079632679},
            {"panda_joint7", 0.785398163397}};

    move_group.setJointValueTarget(home);
    bool success = (move_group.plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
}

void goHome2(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan plan_arm)
{
   std::map<std::string, double> home;
   home = {{"panda_joint1", 1.57079632679},
           {"panda_joint2", -0.785398163397},
           {"panda_joint3", 0},
           {"panda_joint4", -2.35619449019},
           {"panda_joint5", 0},
           {"panda_joint6", 1.57079632679},
           {"panda_joint7", 0.785398163397}};

    move_group.setJointValueTarget(home);
    bool success = (move_group.plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
}

void pre_dump(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan plan_arm)
{
   std::map<std::string, double> home;
   double joint1 = -9;
   double joint2 = 41;
   double joint3 = -23;
   double joint4 = -105;
   double joint5 = 70;
   double joint6 = 156;
   double joint7 = 133;
   home = {{"panda_joint1", deg_to_rad(joint1)},
           {"panda_joint2", deg_to_rad(joint2)},
           {"panda_joint3", deg_to_rad(joint3)},
           {"panda_joint4", deg_to_rad(joint4)},
           {"panda_joint5", deg_to_rad(joint5)},
           {"panda_joint6", deg_to_rad(joint6) },
           {"panda_joint7", deg_to_rad(joint7)}};

    move_group.setJointValueTarget(home);
    bool success = (move_group.plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
}

void dump(moveit::planning_interface::MoveGroupInterface& move_group, moveit::planning_interface::MoveGroupInterface::Plan plan_arm)
{
  std::map<std::string, double> home;
  std::string main_share_dir = ros::package::getPath("panda_pnp_srvcli");
  std::ifstream config_file(main_share_dir + "/json/poses.json");
  if (!config_file.is_open())
  {
      std::cout<<"failed to read config json file"<<std::endl;
  }
  Json::Reader reader;
  Json::Value pose_config;
  reader.parse(config_file, pose_config);
  double joint1 = pose_config["joint_dump"]["joint1"].asFloat();;
  double joint2 = pose_config["joint_dump"]["joint2"].asFloat();;
  double joint3 = pose_config["joint_dump"]["joint3"].asFloat();;
  double joint4 = pose_config["joint_dump"]["joint4"].asFloat();;
  double joint5 = pose_config["joint_dump"]["joint5"].asFloat();;
  double joint6 = pose_config["joint_dump"]["joint6"].asFloat();;
  double joint7 = pose_config["joint_dump"]["joint7"].asFloat();;
  home = {{"panda_joint1", deg_to_rad(joint1)},
          {"panda_joint2", deg_to_rad(joint2)},
          {"panda_joint3", deg_to_rad(joint3)},
          {"panda_joint4", deg_to_rad(joint4)},
          {"panda_joint5", deg_to_rad(joint5)},
          {"panda_joint6", deg_to_rad(joint6) },
          {"panda_joint7", deg_to_rad(joint7)}};

  move_group.setJointValueTarget(home);
  bool success = (move_group.plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group.move();
}

// The addCollisionObjects function creates two tables as collision objects with defined dimensions and positions in the scene.
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.65;
  collision_objects[0].primitives[0].dimensions[1] = 1.525;
  collision_objects[0].primitives[0].dimensions[2] = 0.715;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.11;
  collision_objects[0].primitive_poses[0].position.y = 0.32;
  collision_objects[0].primitive_poses[0].position.z = -0.39;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.65;
  collision_objects[1].primitives[0].dimensions[1] = 1.525;
  collision_objects[1].primitives[0].dimensions[2] = 0.715;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.4;
  collision_objects[1].primitive_poses[0].position.y = -0.78;
  collision_objects[1].primitive_poses[0].position.z = -0.39;

  tf2::Quaternion obj_orientation;
  obj_orientation.setRPY(0, 0, -M_PI / 2);  // A quarter turn about the x-axis and the z-axis
  collision_objects[1].primitive_poses[0].orientation = tf2::toMsg(obj_orientation);
  collision_objects[1].operation = collision_objects[1].ADD;

  // // Add the object on the table to avoid collision
  // collision_objects[2].id = "screen";
  // collision_objects[2].header.frame_id = "panda_link0";

  // /* Define the primitive and its dimensions. */
  // collision_objects[2].primitives.resize(1);
  // collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  // collision_objects[2].primitives[0].dimensions.resize(3);
  // collision_objects[2].primitives[0].dimensions[0] = 0.30;
  // collision_objects[2].primitives[0].dimensions[1] = 0.65;
  // collision_objects[2].primitives[0].dimensions[2] = 1;

  // /* Define the pose of the object. */
  // collision_objects[2].primitive_poses.resize(1);
  // collision_objects[2].primitive_poses[0].position.x = -0.32;
  // collision_objects[2].primitive_poses[0].position.y = 0.71;
  // collision_objects[2].primitive_poses[0].position.z = 0;
  // collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  // collision_objects[2].operation = collision_objects[1].ADD;

  // // Add the wall 
  // collision_objects[3].id = "wall";
  // collision_objects[3].header.frame_id = "panda_link0";

  // /* Define the primitive and its dimensions. */
  // collision_objects[3].primitives.resize(1);
  // collision_objects[3].primitives[0].type = collision_objects[2].primitives[0].BOX;
  // collision_objects[3].primitives[0].dimensions.resize(3);
  // collision_objects[3].primitives[0].dimensions[0] = 0.40;
  // collision_objects[3].primitives[0].dimensions[1] = 1.65;
  // collision_objects[3].primitives[0].dimensions[2] = 1.5;

  // /* Define the pose of the person. */
  // collision_objects[3].primitive_poses.resize(1);
  // collision_objects[3].primitive_poses[0].position.x = -0.65;
  // collision_objects[3].primitive_poses[0].position.y = 0;
  // collision_objects[3].primitive_poses[0].position.z = 0;
  // collision_objects[3].primitive_poses[0].orientation.w = 1.0;

  // collision_objects[3].operation = collision_objects[1].ADD;


  planning_scene_interface.applyCollisionObjects(collision_objects);
}


bool pnp_execute(panda_pnp_srvcli::PnpRequest::Request &req, panda_pnp_srvcli::PnpRequest::Response &res){

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "panda_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "panda_hand";

    // read poses from json file
    std::string main_share_dir = ros::package::getPath("panda_pnp_srvcli");
    std::ifstream config_file(main_share_dir + "/json/poses.json");
    if (!config_file.is_open())
    {
        std::cout<<"failed to read config json file"<<std::endl;
        return false;
    }
    Json::Reader reader;
    Json::Value pose_config;
    reader.parse(config_file, pose_config);

    // const double eef_step = pose_config["step"].asFloat();
    // const double jump_threshold = pose_config["jump"].asFloat();
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
    const double max_vel_factor = pose_config["vel"].asFloat();
    const double max_acc_factor = pose_config["acc"].asFloat();
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    // modify the speed and acceleration of the robot
    move_group_interface_arm.setMaxVelocityScalingFactor(max_vel_factor);
    move_group_interface_arm.setMaxAccelerationScalingFactor(max_acc_factor);


    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // add collision objects to the scene
    addCollisionObjects(planning_scene_interface);

    //  planning interface for the robot arm
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
    // 1. Move to home position
    goHome(move_group_interface_arm, my_plan_arm);

    // 2. Open the gripper
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
    openGripper(move_group_interface_gripper, my_plan_gripper);

    // 3. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("panda_link8");
    std::vector<geometry_msgs::Pose> waypoints1;
    geometry_msgs::Pose target_pose1;
    
    // 4. Move the TCP close to the object
    // 4. Move the TCP close to the object
    if (req.pipe == "long" || req.pipe == "short"){
      if (req.pipe == "long"){
        std::cout<<"move near long pipe"<<std::endl;

        target_pose1.position.x = pose_config["long_prepick_position"]["x"].asFloat();
        target_pose1.position.y = pose_config["long_prepick_position"]["y"].asFloat();
        target_pose1.position.z = pose_config["long_prepick_position"]["z"].asFloat();
        target_pose1.orientation.x = pose_config["long_prepick_orientation"]["x"].asFloat();
        target_pose1.orientation.y = pose_config["long_prepick_orientation"]["y"].asFloat();
        target_pose1.orientation.z = pose_config["long_prepick_orientation"]["z"].asFloat();
        target_pose1.orientation.w = pose_config["long_prepick_orientation"]["w"].asFloat();

        waypoints1.push_back(target_pose1);

        target_pose1.position.x = pose_config["long_pick_position"]["x"].asFloat();
        target_pose1.position.y = pose_config["long_pick_position"]["y"].asFloat();
        target_pose1.position.z = pose_config["long_pick_position"]["z"].asFloat();
        target_pose1.orientation.x = pose_config["long_pick_orientation"]["x"].asFloat();
        target_pose1.orientation.y = pose_config["long_pick_orientation"]["y"].asFloat();
        target_pose1.orientation.z = pose_config["long_pick_orientation"]["z"].asFloat();
        target_pose1.orientation.w = pose_config["long_pick_orientation"]["w"].asFloat();

        waypoints1.push_back(target_pose1);

      }
      else if (req.pipe == "short"){
        std::cout<<"move near short pipe"<<std::endl;

        target_pose1.position.x = pose_config["short_prepick_position"]["x"].asFloat();
        target_pose1.position.y = pose_config["short_prepick_position"]["y"].asFloat();
        target_pose1.position.z = pose_config["short_prepick_position"]["z"].asFloat();
        target_pose1.orientation.x = pose_config["short_prepick_orientation"]["x"].asFloat();
        target_pose1.orientation.y = pose_config["short_prepick_orientation"]["y"].asFloat();
        target_pose1.orientation.z = pose_config["short_prepick_orientation"]["z"].asFloat();
        target_pose1.orientation.w = pose_config["short_prepick_orientation"]["w"].asFloat();

        waypoints1.push_back(target_pose1);

        target_pose1.position.x = pose_config["short_pick_position"]["x"].asFloat();
        target_pose1.position.y = pose_config["short_pick_position"]["y"].asFloat();
        target_pose1.position.z = pose_config["short_pick_position"]["z"].asFloat();
        target_pose1.orientation.x = pose_config["short_pick_orientation"]["x"].asFloat();
        target_pose1.orientation.y = pose_config["short_pick_orientation"]["y"].asFloat();
        target_pose1.orientation.z = pose_config["short_pick_orientation"]["z"].asFloat();
        target_pose1.orientation.w = pose_config["short_pick_orientation"]["w"].asFloat();

        waypoints1.push_back(target_pose1);
      }
    moveit_msgs::RobotTrajectory trajectory1;

    move_group_interface_arm.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory1);

    move_group_interface_arm.execute(trajectory1);


    // 5. Close the  gripper
    double max_effort = 3;    // gripping force (N) 

    closedGripper(move_group_interface_gripper, my_plan_gripper);


    // 6. Move up the gripper
    std::vector<geometry_msgs::Pose> waypoints2;
    target_pose1.position.z = 0.10;
    target_pose1.position.y = target_pose1.position.y - 0.08;
    waypoints2.push_back(target_pose1);

    moveit_msgs::RobotTrajectory trajectory2;
    move_group_interface_arm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
    move_group_interface_arm.execute(trajectory2);

    goHome(move_group_interface_arm, my_plan_arm);

    // 7. Move the TCP above
    std::vector<geometry_msgs::Pose> waypoints3;

    geometry_msgs::Pose target_pose2;

      target_pose2.position.x = pose_config["preplace_position"]["x"].asFloat();
      target_pose2.position.y = pose_config["preplace_position"]["y"].asFloat();
      target_pose2.position.z = pose_config["preplace_position"]["z"].asFloat();
      target_pose2.orientation.x = pose_config["preplace_orientation"]["x"].asFloat();
      target_pose2.orientation.y = pose_config["preplace_orientation"]["y"].asFloat();
      target_pose2.orientation.z = pose_config["preplace_orientation"]["z"].asFloat();
      target_pose2.orientation.w = pose_config["preplace_orientation"]["w"].asFloat();
      waypoints3.push_back(target_pose2);

      moveit_msgs::RobotTrajectory trajectory3;
      move_group_interface_arm.computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory3);
      move_group_interface_arm.execute(trajectory3);


    // 8. Lower the TCP to put down the object
     std::vector<geometry_msgs::Pose> waypoints4;

    geometry_msgs::Pose target_pose3;
    target_pose3.position.x = pose_config["pipe_place_position"]["x"].asFloat();
    target_pose3.position.y = pose_config["pipe_place_position"]["y"].asFloat();
    target_pose3.position.z = pose_config["pipe_place_position"]["z"].asFloat();
    target_pose3.orientation.x = pose_config["pipe_place_orientation"]["x"].asFloat();
    target_pose3.orientation.y = pose_config["pipe_place_orientation"]["y"].asFloat();
    target_pose3.orientation.z = pose_config["pipe_place_orientation"]["z"].asFloat();
    target_pose3.orientation.w = pose_config["pipe_place_orientation"]["w"].asFloat();
    waypoints4.push_back(target_pose3);

    moveit_msgs::RobotTrajectory trajectory4;
    move_group_interface_arm.computeCartesianPath(waypoints4, eef_step, jump_threshold, trajectory4);

    move_group_interface_arm.execute(trajectory4);

    // 9. Open the gripper
    openGripper(move_group_interface_gripper, my_plan_gripper);

    // 10. back to home position
    std::vector<geometry_msgs::Pose> waypoints5;

    geometry_msgs::Pose target_pose4;

    target_pose4.position.x = pose_config["postplace_position"]["x"].asFloat();
    target_pose4.position.y = pose_config["postplace_position"]["y"].asFloat();
    target_pose4.position.z = pose_config["postplace_position"]["z"].asFloat();
    target_pose4.orientation.x = pose_config["postplace_orientation"]["x"].asFloat();
    target_pose4.orientation.y = pose_config["postplace_orientation"]["y"].asFloat();
    target_pose4.orientation.z = pose_config["postplace_orientation"]["z"].asFloat();
    target_pose4.orientation.w = pose_config["postplace_orientation"]["w"].asFloat();
    waypoints5.push_back(target_pose4);

    moveit_msgs::RobotTrajectory trajectory5;
    move_group_interface_arm.computeCartesianPath(waypoints5, eef_step, jump_threshold, trajectory5);
    move_group_interface_arm.execute(trajectory5);

    goHome(move_group_interface_arm, my_plan_arm);

    res.result = "success";
    return true;
    }

    else if (req.pipe == "joint"){

        goHome2(move_group_interface_arm, my_plan_arm);

        target_pose1.position.x = pose_config["joint_prepick_position"]["x"].asFloat();
        target_pose1.position.y = pose_config["joint_prepick_position"]["y"].asFloat();
        target_pose1.position.z = pose_config["joint_prepick_position"]["z"].asFloat();
        target_pose1.orientation.x = pose_config["joint_prepick_orientation"]["x"].asFloat();
        target_pose1.orientation.y = pose_config["joint_prepick_orientation"]["y"].asFloat();
        target_pose1.orientation.z = pose_config["joint_prepick_orientation"]["z"].asFloat();
        target_pose1.orientation.w = pose_config["joint_prepick_orientation"]["w"].asFloat();

        waypoints1.push_back(target_pose1);

        target_pose1.position.z -= 0.15;
        // target_pose1.position.x += 0.05;

        waypoints1.push_back(target_pose1);

        moveit_msgs::RobotTrajectory trajectory1;

        // compute the cartesian path
        move_group_interface_arm.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory1);
        move_group_interface_arm.execute(trajectory1);     // execute the trajectory

        // Close the  gripper
        double max_effort = 3;  // gripping force (N) 
        closedGripper(move_group_interface_gripper, my_plan_gripper);

        // pick it up 
        std::vector<geometry_msgs::Pose> waypoints2;
        target_pose1.position.z += 0.30;
        target_pose1.position.x += 0.05;
        waypoints2.push_back(target_pose1);

        moveit_msgs::RobotTrajectory trajectory2;
        move_group_interface_arm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
        move_group_interface_arm.execute(trajectory2);


        // go to preplace location
        // goHome2(move_group_interface_arm, my_plan_arm);
        goHome(move_group_interface_arm, my_plan_arm);
        // pre_dump(move_group_interface_arm, my_plan_arm);
        dump(move_group_interface_arm, my_plan_arm);
        goHome(move_group_interface_arm, my_plan_arm);

        goHome2(move_group_interface_arm, my_plan_arm);

        std::vector<geometry_msgs::Pose> waypoints6;

        geometry_msgs::Pose target_pose5;
        target_pose5.position.x = pose_config["joint_prepick_position"]["x"].asFloat();
        target_pose5.position.y = pose_config["joint_prepick_position"]["y"].asFloat();
        target_pose5.position.z = pose_config["joint_prepick_position"]["z"].asFloat();
        target_pose5.orientation.x = pose_config["joint_prepick_orientation"]["x"].asFloat();
        target_pose5.orientation.y = pose_config["joint_prepick_orientation"]["y"].asFloat();
        target_pose5.orientation.z = pose_config["joint_prepick_orientation"]["z"].asFloat();
        target_pose5.orientation.w = pose_config["joint_prepick_orientation"]["w"].asFloat();

        waypoints6.push_back(target_pose5);

        target_pose5.position.z -= 0.15;
        // target_pose1.position.x += 0.05;

        waypoints6.push_back(target_pose5);

        moveit_msgs::RobotTrajectory trajectory6;

        // compute the cartesian path
        move_group_interface_arm.computeCartesianPath(waypoints6, eef_step, jump_threshold, trajectory6);
        move_group_interface_arm.execute(trajectory6);     // execute the trajectory

        openGripper(move_group_interface_gripper, my_plan_gripper);

        std::vector<geometry_msgs::Pose> waypoints7;
        target_pose5.position.z += 0.30;
        target_pose5.position.x += 0.05;
        waypoints7.push_back(target_pose5);

        moveit_msgs::RobotTrajectory trajectory7;
        move_group_interface_arm.computeCartesianPath(waypoints7, eef_step, jump_threshold, trajectory7);
        move_group_interface_arm.execute(trajectory7);

        goHome(move_group_interface_arm, my_plan_arm);

        res.result = "success";
        return true;

    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_pnp_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("panda_pnp_service", pnp_execute);

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}