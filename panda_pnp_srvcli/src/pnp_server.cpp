
/*
This C++ program initiates a pick-and-place task which controls a Franka robot arm and a Franka gripper
 through the MoveIt and actionlib libraries. 
By sending a request of "short" or "long", it will pick up the desired pipe for you.

      It can be launched by the following command:
        roslaunch panda_pnp_srvcli pick_place.launch 
*/
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <panda_pnp_srvcli/PnpRequest.h>


// read predefined poses in json
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <ros/package.h>

// #include <ament_index_cpp/get_package_share_directory.hpp>


// action client templates for the Franka gripper's grasp and move actions.
typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> GraspClient; 
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction> MoveClient; 

double deg_to_rad(double deg){
  return deg/180 * M_PI;
}
// The openGripper function is a void function that takes a MoveClient object as its parameter.
// This function is responsible for opening the gripper of the Franka robot using the franka_gripper package's MoveAction action client.
void openGripper(MoveClient& gripper_client)
{
  franka_gripper::MoveGoal goal;
  goal.width = 0.06;
  goal.speed = 0.1;

  gripper_client.sendGoal(goal);
  while(!gripper_client.waitForResult(ros::Duration(10.0))){
    ROS_INFO("waiting for open client");
  }
  franka_gripper::MoveResult result = *(gripper_client.getResult());
  ROS_INFO("Success: %d", result.success);
  std::cout<< "gripper opened" <<std::endl;
}

// The closedGripper function is a void function that takes a MoveClient object as its parameter.
// This function is responsible for closing the gripper of the Franka robot using the franka_gripper package's MoveAction action client.
void closedGripper(GraspClient& gripper_client, double effort)
{
  franka_gripper::GraspGoal goal;
  goal.width = 0.02;
  goal.epsilon.inner = 0.02;
  goal.epsilon.outer = 0.02;
  goal.speed = 0.1;
  goal.force = effort;

  gripper_client.sendGoal(goal);
  while(!gripper_client.waitForResult(ros::Duration(10.0))){
    ROS_INFO("waiting for close client");
  }
  franka_gripper::GraspResult result = *(gripper_client.getResult());
  ROS_INFO("Success: %d", result.success);
  std::cout<< "gripper closed" <<std::endl;
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

    // if (req.pipe != "long" || req.pipe != "short"){
    //   std::cout<< "available options: long or short" <<std::endl;
    //   return false;
    // }

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
    // int reg_pos_count = reg_cfg["reg_pos_count"].asInt();

    const double eef_step = pose_config["step"].asFloat();
    const double jump_threshold = pose_config["jump"].asFloat();
    const double max_vel_factor = pose_config["vel"].asFloat();
    const double max_acc_factor = pose_config["acc"].asFloat();

    static const std::string PLANNING_GROUP_ARM = "panda_arm";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    // Create action clients for the gripper
    GraspClient grasp_client("franka_gripper/grasp", true);
    MoveClient move_client("franka_gripper/move", true);
    grasp_client.waitForServer(ros::Duration(5.0));
    move_client.waitForServer(ros::Duration(5.0));

    // modify the speed and acceleration of the robot
    move_group_interface_arm.setMaxVelocityScalingFactor(max_vel_factor);
    move_group_interface_arm.setMaxAccelerationScalingFactor(max_acc_factor);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // add collision objects to the scene
    addCollisionObjects(planning_scene_interface);

    //  planning interface for the robot arm
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
    robot_trajectory::RobotTrajectory r_trajec(move_group_interface_arm.getRobotModel(), PLANNING_GROUP_ARM);

    

    // 1. Move to home position
    std::cout<<"moving to home"<<std::endl;
    goHome(move_group_interface_arm, my_plan_arm);

    // 2. Open the gripper
    std::cout<<"opening gripper"<<std::endl;

    openGripper(move_client);

    // 3. Place the TCP (Tool Center Point, the tip of the robot) near the object
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("panda_link8");

    std::vector<geometry_msgs::Pose> waypoints1;
    geometry_msgs::Pose target_pose1;

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

      // compute the cartesian path
      move_group_interface_arm.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory1);

      // slow down the trajectory
      
      r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory1);
      iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
      r_trajec.getRobotTrajectoryMsg(trajectory1);

      move_group_interface_arm.execute(trajectory1);     // execute the trajectory

      // 5. Close the  gripper
      double max_effort = 3;  // gripping force (N) 
      closedGripper(grasp_client, max_effort);  

      // 6. Move up the gripper
      std::vector<geometry_msgs::Pose> waypoints2;
      target_pose1.position.z = 0.10;
      target_pose1.position.y = target_pose1.position.y - 0.08;
      waypoints2.push_back(target_pose1);

      moveit_msgs::RobotTrajectory trajectory2;
      move_group_interface_arm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

      r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory2);
      iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
      r_trajec.getRobotTrajectoryMsg(trajectory2);
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
      r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory3);
      iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
      r_trajec.getRobotTrajectoryMsg(trajectory3);
      move_group_interface_arm.execute(trajectory3);

      // 8. Move the TCP near the person
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
      r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory4);
      iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
      r_trajec.getRobotTrajectoryMsg(trajectory4);
      move_group_interface_arm.execute(trajectory4);

      // 9. Open the gripper
      openGripper(move_client);

      // 10. Move back to Home
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
      r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory5);
      iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
      r_trajec.getRobotTrajectoryMsg(trajectory5);
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

        // slow down the trajectory
        r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory1);
        iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
        r_trajec.getRobotTrajectoryMsg(trajectory1);

        move_group_interface_arm.execute(trajectory1);     // execute the trajectory

        // Close the  gripper
        double max_effort = 3;  // gripping force (N) 
        closedGripper(grasp_client, max_effort);  

        // pick it up 
        std::vector<geometry_msgs::Pose> waypoints2;
        target_pose1.position.z += 0.30;
        target_pose1.position.x += 0.05;
        waypoints2.push_back(target_pose1);

        moveit_msgs::RobotTrajectory trajectory2;
        move_group_interface_arm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

        r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory2);
        iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
        r_trajec.getRobotTrajectoryMsg(trajectory2);
        move_group_interface_arm.execute(trajectory2);


        // go to preplace location
        // goHome2(move_group_interface_arm, my_plan_arm);
        goHome(move_group_interface_arm, my_plan_arm);
        // pre_dump(move_group_interface_arm, my_plan_arm);
        dump(move_group_interface_arm, my_plan_arm);
        goHome(move_group_interface_arm, my_plan_arm);

        // std::vector<geometry_msgs::Pose> waypoints10;

        // geometry_msgs::Pose target_pose10;

        // target_pose10.position.x = pose_config["joint_place_position"]["x"].asFloat();
        // target_pose10.position.y = pose_config["joint_place_position"]["y"].asFloat();
        // target_pose10.position.z = pose_config["joint_place_position"]["z"].asFloat();
        // target_pose10.orientation.x = pose_config["joint_place_orientation"]["x"].asFloat();
        // target_pose10.orientation.y = pose_config["joint_place_orientation"]["y"].asFloat();
        // target_pose10.orientation.z = pose_config["joint_place_orientation"]["z"].asFloat();
        // target_pose10.orientation.w = pose_config["joint_place_orientation"]["w"].asFloat();
        // waypoints10.push_back(target_pose10);

        // moveit_msgs::RobotTrajectory trajectory10;
        // move_group_interface_arm.computeCartesianPath(waypoints10, eef_step, jump_threshold, trajectory10);
        // r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory10);
        // iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
        // r_trajec.getRobotTrajectoryMsg(trajectory10);
        // move_group_interface_arm.execute(trajectory10);


        // // dump the joint into the bucket
        // std::vector<geometry_msgs::Pose> waypoints5;

        // geometry_msgs::Pose target_pose4;

        // target_pose4.position.x = pose_config["joint_postplace_position"]["x"].asFloat();
        // target_pose4.position.y = pose_config["joint_postplace_position"]["y"].asFloat();
        // target_pose4.position.z = pose_config["joint_postplace_position"]["z"].asFloat();
        // target_pose4.orientation.x = pose_config["joint_postplace_orientation"]["x"].asFloat();
        // target_pose4.orientation.y = pose_config["joint_postplace_orientation"]["y"].asFloat();
        // target_pose4.orientation.z = pose_config["joint_postplace_orientation"]["z"].asFloat();
        // target_pose4.orientation.w = pose_config["joint_postplace_orientation"]["w"].asFloat();
        // waypoints5.push_back(target_pose4);

        // moveit_msgs::RobotTrajectory trajectory5;
        // move_group_interface_arm.computeCartesianPath(waypoints5, eef_step, jump_threshold, trajectory5);
        // r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory5);
        // iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
        // r_trajec.getRobotTrajectoryMsg(trajectory5);
        // move_group_interface_arm.execute(trajectory5);

        // goHome(move_group_interface_arm, my_plan_arm);

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

        // slow down the trajectory
        r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory6);
        iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
        r_trajec.getRobotTrajectoryMsg(trajectory6);

        move_group_interface_arm.execute(trajectory6);     // execute the trajectory

        openGripper(move_client);

        std::vector<geometry_msgs::Pose> waypoints7;
        target_pose5.position.z += 0.30;
        target_pose5.position.x += 0.05;
        waypoints7.push_back(target_pose5);

        moveit_msgs::RobotTrajectory trajectory7;
        move_group_interface_arm.computeCartesianPath(waypoints7, eef_step, jump_threshold, trajectory7);

        r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory7);
        iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
        r_trajec.getRobotTrajectoryMsg(trajectory7);
        move_group_interface_arm.execute(trajectory7);


        // go to preplace location
        // goHome2(move_group_interface_arm, my_plan_arm);
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