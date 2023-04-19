cmake_minimum_required(VERSION 3.0.2)
project(panda_pick_and_place)

find_package(catkin REQUIRED 
COMPONENTS
  moveit_tutorials
  roscpp
  message_generation
  control_msgs
  actionlib
  sensor_msgs
  xmlrpcpp
  actionlib_msgs
  franka_gripper

)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES panda_pick_and_place
 CATKIN_DEPENDS 
  moveit_tutorials
  roscpp
  message_generation
  control_msgs
  actionlib
  sensor_msgs
  xmlrpcpp
  actionlib_msgs
  franka_gripper
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
add_executable(pnp_long src/pnp_long.cpp)
add_executable(pnp_short src/pnp_short.cpp)
add_executable(pnp_sim src/pnp_sim.cpp)

target_link_libraries(pnp_long ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(pnp_short ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(pnp_sim ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS pnp_long pnp_short pnp_sim  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
