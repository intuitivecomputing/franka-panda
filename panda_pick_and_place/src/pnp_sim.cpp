
/*
This C++ program uses the MoveIt! library to simulate the Franka gripper performing a pick-and-place action.
The purpose of this executable is to verify the trajectory and the pose of the collision objects.

      It can be launched by the following command:
        roslaunch panda_pick_and_place pick_place_sim.launch 
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

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

// The addCollisionObjects function creates two tables as collision objects with defined dimensions and positions in the scene.
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table 
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

  // Add the second table 
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
  collision_objects[1].primitive_poses[0].position.x = -0.77;
  collision_objects[1].primitive_poses[0].position.y = 0.32;
  collision_objects[1].primitive_poses[0].position.z = -0.39;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Add the object on the table to avoid collision
  collision_objects[2].id = "screen";
  collision_objects[2].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.30;
  collision_objects[2].primitives[0].dimensions[1] = 0.65;
  collision_objects[2].primitives[0].dimensions[2] = 1.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = -0.32;
  collision_objects[2].primitive_poses[0].position.y = 0.71;
  collision_objects[2].primitive_poses[0].position.z = 0;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[1].ADD;

  // Add the operator 
  collision_objects[3].id = "person";
  collision_objects[3].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.40;
  collision_objects[3].primitives[0].dimensions[1] = 0.65;
  collision_objects[3].primitives[0].dimensions[2] = 1.5;

  /* Define the pose of the person. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.25;
  collision_objects[3].primitive_poses[0].position.y = 0.65;
  collision_objects[3].primitive_poses[0].position.z = 0;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;

  collision_objects[3].operation = collision_objects[1].ADD;


  planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle n;
    
    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "panda_arm";
    static const std::string PLANNING_GROUP_GRIPPER = "panda_hand";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

    // modify the speed and acceleration of the robot
    move_group_interface_arm.setMaxVelocityScalingFactor(0.5);
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.5);

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


    // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("panda_link8");
    geometry_msgs::Pose target_pose1;
  
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = -0.6;
    target_pose1.position.y = 0.1;
    target_pose1.position.z = 0.22;
    move_group_interface_arm.setPoseTarget(target_pose1);

    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();


    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    // 3. Open the gripper
    openGripper(move_group_interface_gripper, my_plan_gripper);

    // 4. Move the TCP close to the object
    target_pose1.position.z = target_pose1.position.z - 0.1;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // 5. Close the  gripper
    double max_effort = 3;    // gripping force (N) 

    closedGripper(move_group_interface_gripper, my_plan_gripper);


    // 6. Move the TCP above the plate
    target_pose1.position.z = target_pose1.position.z + 0.2;
    target_pose1.position.x = target_pose1.position.x + 0.5;
    target_pose1.position.y = target_pose1.position.y + 0.2;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // 7. Lower the TCP above the plate
    target_pose1.position.z = target_pose1.position.z - 0.2;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // 8. Open the gripper
    openGripper(move_group_interface_gripper, my_plan_gripper);

    // 9. back to home position
    goHome(move_group_interface_arm, my_plan_arm);


    ros::shutdown();
    return 0;
}