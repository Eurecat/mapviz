cmake_minimum_required(VERSION 2.8.3)
project(grape_msgs)

set(MSG_DEPS
    geometry_msgs
    std_msgs
    actionlib_msgs
    moveit_msgs)

find_package(catkin REQUIRED message_generation ${MSG_DEPS} )

add_message_files( FILES
  ManipulationStatus.msg
  RoutePoint.msg
  DeploymentPose.msg
  VirtualFence.msg
  VirtualFences.msg
)

add_service_files( FILES
  DeleteRoute.srv
  GetRoute.srv
  PlanRoute.srv
)

add_action_files( FILES
  ArmManipulation.action
  MoveArmToPose.action
  PlaceDispenser.action
  ScanPlant.action
  CreateDeploymentPoint.action
  SelectPointManually.action
)

generate_messages( DEPENDENCIES ${MSG_DEPS} )

catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
  )
