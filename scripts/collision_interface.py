#!/usr/bin/env python3
import copy
import sys
from math import pi
import numpy as np
from geometry_msgs.msg import Pose, TransformStamped,PoseStamped
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
# from moveit_msgs.msg import CollisionObject
import rospy
import tf2_ros
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from arm_control.srv import collision_object_srv, collision_object_srvRequest,\
                             collision_object_srvResponse, DummyMarker



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

MAX_MID_PANEL_ARUCO_ID=4
# MAX_MID_PANEL_ARUCO_ID=9
# class Box:
#   def __init__(self):
#     self.name=""
#     self.pose=Pose()
#     self.size=[]












class CollisionInterface:
  def __init__(self):
    # OBJECTS_NUMBER=10
    # self.objects=[Box()]*OBJECTS_NUMBER
    self.scene= moveit_commander.PlanningSceneInterface()
    self.robot=moveit_commander.RobotCommander()
    self.GROUP_NAME="manipulator"
    self.move_group = moveit_commander.MoveGroupCommander(self.GROUP_NAME)
    # planning_frame=self.move_group.get_planning_frame()#:world==base_link
    # pose_reference_frame=self.move_group.get_pose_reference_frame()#:world==base_link
    self.BASE_FRAME="base_link"
    self.TABLE_CREATED=False
    self.ROBOT_FRAME_CREATED=False
    self.MID_PANEL_CREATED=False
    self.BUTTONS_CREATED=[False]*MAX_MID_PANEL_ARUCO_ID
    self.LEFT_PANEL_CREATED=False
    self.IMU_CREATED=False
    self.RIGHT_PANEL_CREATED=False
    self.LID_CREATED=False
    self.INSPECTION_PANEL_CREATED=False
    #TODO: check if objects exists at startup
    #   robustness in case of node death
    self.checkExistingObjects()
    self.CLEARANCE_SAFETY_COEF=1.05
    self.tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(1))
    self.tf_listener=tf2_ros.TransformListener(self.tf_buffer,queue_size=None)
    TIMER_DURATION=rospy.Duration(secs=1)
    print('ready to create objects')
    self.update_timer=rospy.Timer(TIMER_DURATION,self.updateObjects)

  def checkExistingObjects(self):
    print('initializing objects states:...')
    #TODO: which one is the better way?
    #   1) check existing objects names
    #      if object_name in self.scene.get_known_object_names() -"_hb"
    #   2) save over param server

  def updateObjects(self,_):
    # known_obj_names=self.scene.get_known_object_names()
    # print(known_obj_names)
    # print(self.scene.get_object_poses(known_obj_names))
    # print(self.scene.get_objects())
    if not self.TABLE_CREATED:
      exists_reference,reference_tf=self.checkObjectReference('table_')
      if exists_reference:
        self.createTablePlane(reference_tf)

    if not self.ROBOT_FRAME_CREATED:
      exists_reference,reference_tf=self.checkObjectReference('robot_frame')
      if exists_reference:
        self.createRobotFrameBox(reference_tf)

    if not self.MID_PANEL_CREATED:
      exists_reference,reference_tf=self.checkObjectReference("mid_panel")
      if exists_reference:
        self.createMidPanelPlane(reference_tf)

    if not self.LEFT_PANEL_CREATED:
      exists_reference,reference_tf=self.checkObjectReference("left_panel")
      if exists_reference:
        self.createLeftPanelPlane(reference_tf)

    if not all(self.BUTTONS_CREATED):
      for not_created in np.where(np.bitwise_not(self.BUTTONS_CREATED))[0]:
        button_reference="button_"+str(not_created+1)
        # print('creating '+button_reference)
        exists_reference,reference_tf=self.checkObjectReference(button_reference)
        if exists_reference:
          self.createButtonBox(reference_tf)
          self.BUTTONS_CREATED[not_created]=True

    if not self.IMU_CREATED:
      exists_reference,reference_tf=self.checkObjectReference("imu_")
      if exists_reference:
        self.createImuBox(reference_tf)

    # if not self.RIGHT_PANEL_CREATED  and exists_base():
    #   RIGHT_PANEL_POSES=[arucoInquiriesClient(id) for id in self.RIGHT_PANEL_MARKERS]
    #   if all([pose.found for pose in RIGHT_PANEL_POSES]):
    #     self.computeRightPanelPose([pose.pose for pose in RIGHT_PANEL_POSES])

    # if not self.LID_CREATED:
    #   LID_POSES=[arucoInquiriesClient(id) for id in self.LID_MARKERS]
    #   if all([pose.found for pose in LID_POSES]):
    #     self.computeLidPose([pose.pose for pose in LID_POSES])

    # if not self.INSPECTION_PANEL_CREATED:
    #   INSPECTION_PANEL_POSES=[arucoInquiriesClient(id) for id in self.INSPECTION_PANEL_MARKERS]
    #   if all([pose.found for pose in INSPECTION_PANEL_POSES]):
    #     self.computeInspectionBoxPose([pose.pose for pose in INSPECTION_PANEL_POSES])

  def checkObjectReference(self,target_frame):
    global tf_buffer
    check_timeout=0
    # has .transform.{translation.{x,y,z},rotation.{x,y,z,w}
    while True:
        try:
            return True, self.tf_buffer.lookup_transform(self.BASE_FRAME,target_frame,rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
            tf2_ros.ExtrapolationException): continue
        finally:
          check_timeout+=1
          if check_timeout>50: return False,None

  def createTablePlane(self,reference_tf):
    print('creating plane for: '+'table_') 
    plane_pose=PoseStamped()
    plane_pose.header=reference_tf.header
    plane_pose.pose.position.x=reference_tf.transform.translation.x
    plane_pose.pose.position.y=reference_tf.transform.translation.y
    plane_pose.pose.position.z=reference_tf.transform.translation.z
    plane_pose.pose.orientation.x=reference_tf.transform.rotation.x
    plane_pose.pose.orientation.y=reference_tf.transform.rotation.y
    plane_pose.pose.orientation.z=reference_tf.transform.rotation.z
    plane_pose.pose.orientation.w=reference_tf.transform.rotation.w
    # self.scene.add_plane("table_hb", plane_pose, normal=(0, 0, 1), offset=0)
    self.scene.add_box("table_hb", plane_pose, 
                size=(self.CLEARANCE_SAFETY_COEF*.75, 
                self.CLEARANCE_SAFETY_COEF*.75, 
                self.CLEARANCE_SAFETY_COEF*.0005))
    
    self.TABLE_CREATED=True

  def createRobotFrameBox(self,reference_tf):
    print('creating box for: '+'robot_frame') 
    box_pose=PoseStamped()
    box_pose.header=reference_tf.header
    box_pose.pose.position.x=reference_tf.transform.translation.x
    box_pose.pose.position.y=reference_tf.transform.translation.y
    box_pose.pose.position.z=reference_tf.transform.translation.z
    box_pose.pose.orientation.x=reference_tf.transform.rotation.x
    box_pose.pose.orientation.y=reference_tf.transform.rotation.y
    box_pose.pose.orientation.z=reference_tf.transform.rotation.z
    box_pose.pose.orientation.w=reference_tf.transform.rotation.w

    self.scene.add_box("robot_frame_hb", box_pose, 
                        size=(self.CLEARANCE_SAFETY_COEF*.75,
                        self.CLEARANCE_SAFETY_COEF* .15, 
                        self.CLEARANCE_SAFETY_COEF*.15))
    self.ROBOT_FRAME_CREATED=True

  def createMidPanelPlane(self,reference_tf):
    print('creating plane for: '+'mid_panel') 
    box_pose=PoseStamped()
    box_pose.header=reference_tf.header
    box_pose.pose.position.x=reference_tf.transform.translation.x
    box_pose.pose.position.y=reference_tf.transform.translation.y
    box_pose.pose.position.z=reference_tf.transform.translation.z
    box_pose.pose.orientation.x=reference_tf.transform.rotation.x
    box_pose.pose.orientation.y=reference_tf.transform.rotation.y
    box_pose.pose.orientation.z=reference_tf.transform.rotation.z
    box_pose.pose.orientation.w=reference_tf.transform.rotation.w

    self.scene.add_box("mid_panel_hb", box_pose,
                         size=(self.CLEARANCE_SAFETY_COEF*.3,
                         self.CLEARANCE_SAFETY_COEF* .5, 
                         self.CLEARANCE_SAFETY_COEF*.0005))
    self.MID_PANEL_CREATED=True


  def createButtonBox(self,reference_tf):
    reference_name=str(reference_tf.child_frame_id)
    print(reference_name)
    print('creating plane for: '+reference_name) 
    box_pose=PoseStamped()
    box_pose.header=reference_tf.header
    box_pose.pose.position.x=reference_tf.transform.translation.x
    box_pose.pose.position.y=reference_tf.transform.translation.y
    box_pose.pose.position.z=reference_tf.transform.translation.z
    box_pose.pose.orientation.x=reference_tf.transform.rotation.x
    box_pose.pose.orientation.y=reference_tf.transform.rotation.y
    box_pose.pose.orientation.z=reference_tf.transform.rotation.z
    box_pose.pose.orientation.w=reference_tf.transform.rotation.w

    # self.scene.add_box(reference_name+"_hb", box_pose, 
    #                           size=(self.CLEARANCE_SAFETY_COEF*.023,
    #                           self.CLEARANCE_SAFETY_COEF* .04, 
    #                           self.CLEARANCE_SAFETY_COEF*.04))
    self.scene.add_cylinder(reference_name+"_hb", box_pose, 
                              height=self.CLEARANCE_SAFETY_COEF*.023,
                              radius=self.CLEARANCE_SAFETY_COEF* .04/2)


  def createLeftPanelPlane(self,reference_tf):
    print('creating plane for: '+'left_panel') 
    box_pose=PoseStamped()
    box_pose.header=reference_tf.header
    box_pose.pose.position.x=reference_tf.transform.translation.x
    box_pose.pose.position.y=reference_tf.transform.translation.y
    box_pose.pose.position.z=reference_tf.transform.translation.z
    box_pose.pose.orientation.x=reference_tf.transform.rotation.x
    box_pose.pose.orientation.y=reference_tf.transform.rotation.y
    box_pose.pose.orientation.z=reference_tf.transform.rotation.z
    box_pose.pose.orientation.w=reference_tf.transform.rotation.w

    self.scene.add_box("left_panel_hb", box_pose, 
                              size=(self.CLEARANCE_SAFETY_COEF*.25,
                              self.CLEARANCE_SAFETY_COEF* .36, 
                              self.CLEARANCE_SAFETY_COEF*.0005))
    self.LEFT_PANEL_CREATED=True


  def createImuBox(self,reference_tf):
    print('creating plane for: '+'imu_') 
    box_pose=PoseStamped()
    box_pose.header=reference_tf.header
    box_pose.pose.position.x=reference_tf.transform.translation.x
    box_pose.pose.position.y=reference_tf.transform.translation.y
    box_pose.pose.position.z=reference_tf.transform.translation.z
    box_pose.pose.orientation.x=reference_tf.transform.rotation.x
    box_pose.pose.orientation.y=reference_tf.transform.rotation.y
    box_pose.pose.orientation.z=reference_tf.transform.rotation.z
    box_pose.pose.orientation.w=reference_tf.transform.rotation.w

    self.scene.add_box("imu_hb", box_pose, 
                        size=(self.CLEARANCE_SAFETY_COEF* .05,
                        self.CLEARANCE_SAFETY_COEF* .1, 
                        self.CLEARANCE_SAFETY_COEF* .05))
    
    self.IMU_CREATED=True



  def createRightPanelBox(self):
    """
      z=13+offset
      y=12+offset(x,y)
      x=12+offset(x,y)
      orientation=12 (13)
      size= fixed
    """
    self.RIGHT_PANEL_CREATED=True


  def createLidBox(self):
    """
      z=13
      y=13+offset(x,y)
      x=13+offset(x,y)
      orientation=?
      size= fixed
    """
    self.createLidHandleBox()
    self.LID_CREATED=True

  def createLidHandleBox(self):
    """
      z=13
      y=13+offset(x,y)
      x=13+offset(x,y)
      orientation=?
      size= fixed
    """


  def createInspectionBox(self):
    # 12, 13
    """
      z=12
      y=average(13+offset(x,y), 12+offset(x,y))
      x=average(13+offset(x,y), 12+offset(x,y))
      orientation=?
      size= using arucos + offsets
    """
    self.INSPECTION_PANEL_CREATED=True



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
  moveit_commander.roscpp_initialize(sys.argv)
  # collisionInterface()
  collisor=CollisionInterface()


  try:
    rospy.spin()
  except rospy.ROSInterruptException or KeyboardInterrupt:
    pass

"""
scene = moveit_commander.PlanningSceneInterface()

https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#adding-objects-to-the-planning-scene

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

scene.add_box(
        name="my_box",
        pose=PoseStamped(
              header=Header(frame_id=robot.get_planning_frame()),
              pose=Pose(
                     position=Point(x=1.2, y=1 ,z=1),
                      orientation=Quaternion(x=0, y=0, z=0, w=1)
        size=(0.1, 0.1, 0.1))

scene.add_plane(
        name="my_plane",
        pose=PoseStamped(
                header=Header(
                        stamp=rospy.Time.now(), 
                        frame_id=robot.get_planning_frame()),
                pose=Pose(
                        position=Point(x=0, y=0 ,z=0), 
                        orientation=Quaternion(x=0, y=0, z=0, w=1))),
        normal=(0,0,1),
        offset=1)
  
        # ax + by + cz + d = 0
        # a,b,c := *normal
        # d := offset
        
psi.add_object(moveit_msgs.msg.CollisionObject())
-> ha piani, forme primitive, sotto-frame,....

https://moveit.picknik.ai/humble/doc/tutorials/planning_around_objects/planning_around_objects.html
/collision_object:          moveit_msgs/CollisionObject
/attached_collision_object:   "             "

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

"""