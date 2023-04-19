
/*
This C++ program initiates a pick-and-place task which controls a Franka robot arm and a Franka gripper
 through the MoveIt and actionlib libraries. 

      It can be launched by the following command:
        roslaunch panda_pick_and_place pick_place.launch pipe:=long
*/
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

// action client templates for the Franka gripper's grasp and move actions.
typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> GraspClient; 
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction> MoveClient; 

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
  std::cout<< "open gripper" <<std::endl;
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
  std::cout<< "close gripper" <<std::endl;
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
  collision_objects[2].primitives[0].dimensions[2] = 1;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = -0.32;
  collision_objects[2].primitive_poses[0].position.y = 0.71;
  collision_objects[2].primitive_poses[0].position.z = 0;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[1].ADD;

  // Add the wall 
  collision_objects[3].id = "wall";
  collision_objects[3].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.40;
  collision_objects[3].primitives[0].dimensions[1] = 1.65;
  collision_objects[3].primitives[0].dimensions[2] = 1.5;

  /* Define the pose of the person. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = -0.65;
  collision_objects[3].primitive_poses[0].position.y = 0;
  collision_objects[3].primitive_poses[0].position.z = 0;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;

  collision_objects[3].operation = collision_objects[1].ADD;


  planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char** argv)
{
    const double eef_step = 0.02;
    const double jump_threshold = 6.0;
    const double max_vel_factor = 0.3;
    const double max_acc_factor = 0.3;
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
    openGripper(move_client);

    // 3. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("panda_link8");
    std::vector<geometry_msgs::Pose> waypoints1;
    geometry_msgs::Pose target_pose1;

    // 4. Move the TCP close to the object
    target_pose1.position.x = 0.4;
    target_pose1.position.y = -0.353;
    target_pose1.position.z = 0.017;
    target_pose1.orientation.x = 0.576;
    target_pose1.orientation.y = -0.263;
    target_pose1.orientation.z = 0.304;
    target_pose1.orientation.w = 0.712;
    waypoints1.push_back(target_pose1);

    moveit_msgs::RobotTrajectory trajectory1;

    // compute the cartesian path
    move_group_interface_arm.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory1);

    // slow down the trajectory
    trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
    robot_trajectory::RobotTrajectory r_trajec(move_group_interface_arm.getRobotModel(), PLANNING_GROUP_ARM);
    r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory1);
    iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
    r_trajec.getRobotTrajectoryMsg(trajectory1);

    move_group_interface_arm.execute(trajectory1);     // execute the trajectory

    // 5. Close the  gripper
    double max_effort = 3;  // gripping force (N) 
    closedGripper(grasp_client, max_effort);  

    // 6. Move up the gripper
    std::vector<geometry_msgs::Pose> waypoints2;
    target_pose1.position.z = 0.15;
    target_pose1.position.y = target_pose1.position.y + 0.08;
    waypoints2.push_back(target_pose1);

    // 7. Move the TCP above

    geometry_msgs::Pose target_pose2;
    target_pose2.position.x = 0.3;
    target_pose2.position.y = 0;
    target_pose2.position.z = 0.5;
    target_pose2.orientation = current_pose.pose.orientation;
    waypoints2.push_back(target_pose2);
    moveit_msgs::RobotTrajectory trajectory2;
    move_group_interface_arm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

    r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory2);
    iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
    r_trajec.getRobotTrajectoryMsg(trajectory2);
    move_group_interface_arm.execute(trajectory2);

    // 8. Move the TCP near the person

    std::vector<geometry_msgs::Pose> waypoints3;

    geometry_msgs::Pose target_pose3;

    target_pose3.position.x = 0.09;
    target_pose3.position.y = 0.5;
    target_pose3.position.z = 0.1;
    target_pose3.orientation = current_pose.pose.orientation;
  
    waypoints3.push_back(target_pose3);
    moveit_msgs::RobotTrajectory trajectory3;
    move_group_interface_arm.computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory3);
    r_trajec.setRobotTrajectoryMsg(*move_group_interface_arm.getCurrentState(), trajectory3);
    iptp.computeTimeStamps(r_trajec, max_vel_factor, max_acc_factor);
    r_trajec.getRobotTrajectoryMsg(trajectory3);
    move_group_interface_arm.execute(trajectory3);

    // 9. Open the gripper
    openGripper(move_client);

    // 10. Move back to Home
    goHome(move_group_interface_arm, my_plan_arm);

    ros::shutdown();
    return 0;
}