#!/usr/bin/env python3
import copy
from pickle import OBJ
import sys
from math import pi
from geometry_msgs.msg import Pose
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from arm_control.srv import (collision_object_srv, collision_object_srvRequest,
                             collision_object_srvResponse)

def callback_service(req):
    global bool_exit
    #print('Collision service received')

    if req.exit==True :
        bool_exit=True
        return True

    if req.add==True:
        #print(req.box_pose)
        #print(req.box_size)
        #print(req.box_name)
        return tutorial.add_box(req.box_pose,box_name=req.box_name,box_size=req.box_size)
    if req.attach==True:
        return tutorial.attach_box(req.box_pose,box_name=req.box_name,box_size=req.box_size)
    if req.detach==True:
        return tutorial.detach_box(req.box_pose,box_name=req.box_name,box_size=req.box_size)
    if req.add==False and req.exit==False:
        return tutorial.remove_box(box_name=req.box_name)

    #return collision_object_srvResponse(success=True)

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

class Box:
  def __init__(self):
    self.name=""
    self.pose=Pose()
    self.size=[]

class CollisionInterface:
  def __init__(self):
    OBJECTS_NUMBER=10
    self.boxes=[Box()]*OBJECTS_NUMBER
    self.scene=""
    self.robot=""


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False


  def add_box(self,box_pose,box_name,box_size, timeout=4):
    #box_name = self.box_name
    scene = self.scene
    #box_pose = geometry_msgs.msg.PoseStamped()
    #box_pose.header.frame_id = "base_link"
    #box_pose.pose.orientation.w = 1.0
    #box_pose.pose.position.x = -0.1 # slightly above the end effector
    #box_name = "box"
    size=(box_size[0],box_size[1],box_size[2])
    scene.add_box(box_name, box_pose, size)
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self,box_pose,box_name,box_size, timeout=4):
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names
    grasping_group = 'manipulator'
    #touch_links = robot.get_link_names()
    touch_links=['wrist_3_link', 'ee_link', 'tool0', 'camera_ur_mount', 'camera_link1', 'camera_link', 'camera_camera_lens', 'camera_camera', 'camera_camera_gazebo', 'robotiq_arg2f_base_link', 'left_outer_knuckle', 'left_outer_finger', 'left_inner_finger', 'left_finger_tip_temp', 'left_finger_tip', 'left_inner_finger2', 'left_inner_knuckle', 'left_inner_knuckle2', 'plate1', 'dys_middle', 'right_inner_knuckle', 'right_inner_knuckle2', 'right_outer_knuckle', 'right_outer_finger', 'right_inner_finger', 'right_finger_tip_temp', 'right_finger_tip', 'right_inner_finger2']
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self,box_pose,box_name,box_size, timeout=4):
    scene = self.scene
    eef_link = self.eef_link
    scene.remove_attached_object(eef_link, name=box_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self,box_name,timeout=4):
    #box_name = self.box_name
    scene = self.scene
    scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)





def collisionInterface():
  #COLLISION SERVER FOR ATTACH/DETACH REQUESTS
  rospy.Service('collision_server', collision_object_srv, callback_service)
  #ARUCO INQUIRER SERVICE to simplify the check over found arucos...
  # rospy.Service()

if __name__ == '__main__':
  node_name="collision_interface"
  rospy.init_node(node_name,anonymous=False)
  collisionInterface()

  
  try:
    rospy.spin()
  except rospy.ROSInterruptException or KeyboardInterrupt:
    pass

"""
https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#adding-objects-to-the-planning-scene

ADDING BOX:
scene = moveit_commander.PlanningSceneInterface()

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = BOX_FRAME_NAME
box_pose.pose.orientation.w = 1.0
...
box_pose.pose.position.z = 0.11
...
box_name = BOX_NAME
scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

#ENSURE CREATION: if node was just created/dies before completing box won't be created
#_check_changes_in_lists w/ timeout:
#  get_attached_objects()
#  get_known_object_names()
start = rospy.get_time()
seconds = rospy.get_time()
while (seconds - start < timeout) and not rospy.is_shutdown():
    #check in attached objects
    attached_objects = scene.get_attached_objects([box_name])
    is_attached = len(attached_objects.keys()) > 0

    #check in the scene.
    #!!attaching the box will remove it from known_objects!!
    is_known = box_name in scene.get_known_object_names()

    #test expected state (can simply use truth values here)
    if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

    rospy.sleep(0.1)#
    seconds = rospy.get_time()
return False#TIMEDOUT


/collision_object
/attached_col...
moveit_msgs/CollisionObject

"""