cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

  find_package(catkin REQUIRED COMPONENTS
    geometric_shapes
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_visual_tools
  moveit_ros_planning_interface
  gazebo_msgs
  pcl_conversions
  pcl_ros
  pluginlib
  rosbag
  roscpp
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  std_msgs
  eigen_conversions
  tf_conversions
  message_generation
  geometry_msgs
  )



 add_message_files(
   FILES
 )

 add_service_files(
   FILES
   GripperCommand.srv
   UserInterface.srv
   aruco_service.srv
   cv_server.srv
   collision_object_srv.srv
   float_return_srv.srv
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

 catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS message_runtime
  )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS} 
    ${THIS_PACKAGE_INCLUDE_DIRS}
  )



# add_executable(ERC src/ERC.cpp)

# add_dependencies(ERC ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# target_link_libraries(ERC ${catkin_LIBRARIES} ${Boost_LIBRARIES})

 catkin_install_python(
  PROGRAMS
   scripts/UI_gara.py
   scripts/collision_interface.py
   scripts/fake_controller.py
   scripts/gripper_controller.py
   QT/user_interface.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


# install(TARGETS  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


