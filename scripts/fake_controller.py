#!/usr/bin/env python3

import rospy
import numpy as np
import os
from geometry_msgs.msg import Pose, TransformStamped
import tf2_ros
from tf.transformations import quaternion_inverse
from arm_control.srv import GripperCommand
from arm_vision.srv import FoundMarker,ReferenceAcquisition
import moveit_commander
from moveit_msgs.msg  import Grasp,DisplayTrajectory
from math import pi as PI
from std_msgs.msg import String,Empty
from moveit_commander.conversions import pose_to_list


from moveit_commander import *
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
import moveit_commander
import copy


class MyMoveGroup(object):
  """Move_group_class"""
  def __init__(self):
    # super(Move_group_class, self).__init__()

    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('state_machine', anonymous=True)
    ## kinematic model and the robot's current joint states
    robot = RobotCommander()
    ## robot's internal understanding of the surrounding world:
    scene = PlanningSceneInterface()
    ## interface to a planning group (group of joints)
    group_name = "manipulator"
    move_group =MoveGroupCommander(group_name)
    group_name = "gripper"
    ee_move_group =MoveGroupCommander(group_name)

    ## display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   DisplayTrajectory,
                                                   queue_size=20)

    
    #reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    # print ("============ Planning frame: %s" + planning_frame)

    # end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    # print ("============ End effector link: %s" + eef_link)

    # list of all the groups in the robot:
    group_names = robot.get_group_names()
   # print "============ Available Planning Groups:", robot.get_group_names()

    #print robot.get_current_state()
    
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.ee_move_group=ee_move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def all_close(self,goal, actual, tolerance):
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

    elif type(goal) is PoseStamped:
      return self.all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
      return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


  def ruota_giunto(self,id_giunto,angle):
    joints=self.get_joints_values()
    joints[id_giunto]=joints[id_giunto]+angle
    self.go_to_joint_state(joints)


  def go_to_joint_state(self,joints_vet):
    move_group = self.move_group

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joints_vet, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return self.all_close(joints_vet, current_joints, 0.01)

  def go_to_pose_goal(self,pose_goal):
    move_group = self.move_group

    # pose_goal = geometry_msgs.msg.Pose()
    move_group.set_pose_target(pose_goal)
    
    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return self.all_close(pose_goal, current_pose, 0.01)


  def go_to_pose_cartesian(self,pose_goal):
  
    move_group = self.move_group

    waypoints = []

    waypoints.append(copy.deepcopy(pose_goal))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    self.execute_plan(plan)
    self.display_trajectory(plan)

    return plan, fraction
    

  def plan_cartesian_path(self, scale=1):
    move_group = self.move_group
    
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    display_trajectory_publisher.publish(display_trajectory)

  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def add_box(self,collision_box, timeout=0):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #box_name = self.box_name
    #print('adding box')
    #print(box_name)
    #print(box_size)
    #print(box_pose)
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    #box_pose = geometry_msgs.msg.PoseStamped()
    #box_pose.header.frame_id = "base_link"
    #box_pose.pose.orientation.w = 1.0
    #box_pose.pose.position.x = -0.1 # slightly above the end effector
    #box_name = "box"
    size=(collision_box.box_size[0],collision_box.box_size[1],collision_box.box_size[2])
    scene.add_box(collision_box.box_name, collision_box.box_pose, size)
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  def attach_box(self,box_pose,box_name,box_size, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'manipulator'
    #touch_links = robot.get_link_names()
    touch_links=['wrist_3_link', 'ee_link', 'tool0', 'camera_ur_mount', 'camera_link1', 'camera_link', 'camera_camera_lens', 'camera_camera', 'camera_camera_gazebo', 'robotiq_arg2f_base_link', 'left_outer_knuckle', 'left_outer_finger', 'left_inner_finger', 'left_finger_tip_temp', 'left_finger_tip', 'left_inner_finger2', 'left_inner_knuckle', 'left_inner_knuckle2', 'plate1', 'dys_middle', 'right_inner_knuckle', 'right_inner_knuckle2', 'right_outer_knuckle', 'right_outer_finger', 'right_inner_finger', 'right_finger_tip_temp', 'right_finger_tip', 'right_inner_finger2']

    #print(touch_links)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
  def detach_box(self,box_pose,box_name,box_size, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
  def remove_box(self,box_name,timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #box_name = self.box_name
    scene = self.scene
    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
  
  def get_joints_values(self):
    #BUG
    # joints_grad=self.move_group.get_current_joint_values()
    joints_grad=move_group.get_current_joint_values()
    return joints_grad

  def FermaRobot(self):
    print("Il robot sta per essere fermato")
    self.move_group.stop()
    self.move_group.clear_pose_targets()

my_move_group=MyMoveGroup()

exit_request=False

INQUIRIES_SERVICE='aruco_inquiries'
GRIPPER_SERVICE='gripper_commands'
REFERENCES_SERVICE='references'
#TODO add auto base_link name retrival
BASE_FRAME='base_link'

#TODO: use this to check if exploration was 100s% successfull
#  must use referencing/objects because aruco always successfully found...
#   maybe it will be better to give the referencing providing ability to aruco_detector?
#   since sometimes the current method does not work
EVERYTHING_FOUND=True

MAX_MID_PANEL_ARUCO_ID=4
REAL_MAX_MID_PANEL_ARUCO_ID=9
#TODO instead of having 2 sources for  required ids, use a service to ask
#referencer node which are the required ones
objects_markers={'table_':            [10,14],
                 'mid_panel':[marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                 'button_1': [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_2':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_3':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_4': [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_5': [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_6':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_7':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_8':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'button_9':  [marker for marker in range (1,MAX_MID_PANEL_ARUCO_ID+1)],
                'imu_':               [10],
                'left_panel':          [11],
                'right_panel':        [12,13],
                'inspection_panel':   [12,13],
                'lid_':               [13],
                'lid_storage':         [14]}

HOME_POSITION=[0,-120,100,20,90,-90]
requested_objective=rospy.set_param("/objective",0)
secret_id=0
tf_buffer=None
tf_listener=None


def grad_to_rad(angle):
    return angle*PI/180


def queryReferenceFramePose(object_name):
  while True:
    try:
      return tf_buffer.lookup_transform(BASE_FRAME,object_name,rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
        tf2_ros.ExtrapolationException): pass
  

#TODO: change it with existing reference and objects
def arucoInquiriesClient(id):
    rospy.wait_for_service(INQUIRIES_SERVICE)
    try:
        inquirer=rospy.ServiceProxy(INQUIRIES_SERVICE,FoundMarker)
        inquiry_result=inquirer(id)
        # if not inquiry_result.found:
        #   print('not found yet')
        # else:
        #   print('found')
        return inquiry_result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def referencesClient(permission_timeout=3):
    rospy.wait_for_service(REFERENCES_SERVICE)
    permission_timeout=int(permission_timeout)
    print("starting referencing service in {} seconds".format(permission_timeout))
    while True:
      try:
        referencer=rospy.ServiceProxy(REFERENCES_SERVICE,ReferenceAcquisition)
        reference_result=referencer(permission_timeout)
        if reference_result.done:
          break
      except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def markersInspection(precision_parameter=0):
  print('workspace exploration')
  group_name="manipulator"
  group=MoveGroupCommander(group_name)
  #motion backwards
  wpose = group.get_current_pose().pose
  wpose.position.x -= 0.2
  my_move_group.go_to_pose_cartesian(wpose)
  referencesClient()
  #rotation to imu
  joint_vet=my_move_group.move_group.get_current_joint_values()
  joint_vet[0]+=grad_to_rad(40)
  joint_vet[1]+=grad_to_rad(20)
  joint_vet[2]+=grad_to_rad(10)
  joint_vet[3]+=grad_to_rad(20)
  my_move_group.go_to_joint_state(joint_vet)
  referencesClient()
  #rising to imu panel
  joint_vet=my_move_group.move_group.get_current_joint_values()
  joint_vet[2]-=grad_to_rad(30)#25
  joint_vet[3]-=grad_to_rad(30)#25
  my_move_group.go_to_joint_state(joint_vet)
  referencesClient()############
  #framing main panel
  wpose = group.get_current_pose().pose
  wpose.position.x -= 0.1
  my_move_group.go_to_pose_cartesian(wpose)
  # referencesClient()
  #rotation to inspection panel
  joint_vet=my_move_group.move_group.get_current_joint_values()
  joint_vet[0]-=grad_to_rad(95)
  joint_vet[2]+=grad_to_rad(15)#0
  joint_vet[3]+=grad_to_rad(15)#0
  my_move_group.go_to_joint_state(joint_vet)
  referencesClient()
  #descending to panel storage
  joint_vet=my_move_group.move_group.get_current_joint_values()
  joint_vet[2]+=grad_to_rad(10)#25
  joint_vet[3]+=grad_to_rad(10)#25
  my_move_group.go_to_joint_state(joint_vet)
  referencesClient()

  #TODO: REPEAT WITH INCREASED MOTION IF NOT EVERYTHING HAS BEEN FOUND
  if not EVERYTHING_FOUND:
    markersInspection(precision_parameter=1)


def gripperCommandClient(gripper_state):
  print("requested gripper state: ",gripper_state)
  try:
    commander=rospy.ServiceProxy(GRIPPER_SERVICE,GripperCommand)
    while True:
      if commander(gripper_state): break
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)


def actuationOfButtons():
    buttons_ids=rospy.get_param('/buttons_sequence')
    if all([arucoInquiriesClient(required).found for required in objects_markers['button_1']]):

    #TODO possibly remove proxy gripper node 
    # print('closing gripper')
    # gripper_pub=rospy.Publisher('/gripper_command',String,queue_size=5)
    # gripper_command=String()
    # gripper_command.data="close"
    # gripper_pub.publish(gripper_command)

      homePositioning()
      gripperCommandClient("close")    

      print('pressing buttons sequence: {}'.format(buttons_ids))
      [pressButton(button_id) for partial in buttons_ids.split()
                            for button_id in partial.split(',') 
                        if not (button_id=='' or button_id==',')]


def pressButton(button_id):
    """print('pressing button: {}'.format(button_id))
    answer=arucoInquiriesClient(int(button_id))
    if answer.found:

      SAFETY_X=0.05
      SAFETY_Y=0
      SAFETY_Z=0

      MARKER_BUTTON_Z_DIST=0.03+0.05/2


      current_pose = group.get_current_pose().pose
      button_pose=answer.pose
      target_pose=current_pose

      # delta_x=button_pose.position.x-current_pose.position.x
      # # >0 move forward +, <0 move backward -
      # delta_y=button_pose.position.y-current_pose.position.y
      # # >0 move right -, <0 move left +
      # delta_z=button_pose.position.z-current_pose.position.z
      # #>0 move up +, <0 move down -
      # target_pose.position.x+=delta_x-SAFETY_X
      # target_pose.orientation=current_pose.orientation
      # my_move_group.go_to_pose_cartesian(button_pose)
      # target_pose.position.y-=delta_y+SAFETY_Y
      # target_pose.orientation=current_pose.orientation
      # my_move_group.go_to_pose_cartesian(button_pose)
      # target_pose.position.z+=delta_z+SAFETY_Z+MARKER_BUTTON_Z_DIST
      # target_pose.orientation=current_pose.orientation
      # my_move_group.go_to_pose_cartesian(button_pose)
      
      delta_x=0.19
      delta_y=-0.05
      delta_z=0.05

      # target_pose.position.x+=delta_x-SAFETY_X
      # target_pose.orientation=current_pose.orientation
      # my_move_group.go_to_pose_cartesian(button_pose)
      # target_pose.position.y-=delta_y+SAFETY_Y
      # target_pose.orientation=current_pose.orientation
      # my_move_group.go_to_pose_cartesian(button_pose)
      # target_pose.position.z+=delta_z+SAFETY_Z+MARKER_BUTTON_Z_DIST
      # target_pose.orientation=current_pose.orientation
      # my_move_group.go_to_pose_cartesian(button_pose)"""


#https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
def sensorPickup():
  if all([arucoInquiriesClient(required).found for required in objects_markers['imu_']]):
    print('picking up IMU module')
    # imu_tf=queryReferenceFramePose('imu_')
    grasp=Grasp()
    grasp.id='imu_grasp'

    # grasp.grasp_pose.pose.position= imu_tf.transform.translation
    # grasp.grasp_pose.pose.orientation= imu_tf.transform.rotation
    grasp.grasp_pose.pose.position.x= -.19
    grasp.grasp_pose.pose.position.y= .115
    grasp.grasp_pose.pose.position.z= .26
    grasp.grasp_pose.pose.orientation.x= -.707
    grasp.grasp_pose.pose.orientation.y= .0
    grasp.grasp_pose.pose.orientation.z= .0
    grasp.grasp_pose.pose.orientation.w= .707
    # grasp.grasp_posture.

    grasp.pre_grasp_approach.direction.header.frame_id='imu_'
    grasp.pre_grasp_approach.direction.vector.x = .0
    grasp.pre_grasp_approach.direction.vector.y = 1
    grasp.pre_grasp_approach.direction.vector.z = .0
    grasp.pre_grasp_approach.min_distance = 0.1
    grasp.pre_grasp_approach.desired_distance = 0.2 
    # grasp.pre_grasp_posture.header.frame_id = "right_gripper_base_link"
    # grasp.pre_grasp_posture.joint_names = ["right_gripper_finger_joint"]
   
    grasp.post_grasp_retreat.direction.header.frame_id='imu_'
    grasp.post_grasp_retreat.direction.vector.x=0
    grasp.post_grasp_retreat.direction.vector.x = .0
    grasp.post_grasp_retreat.direction.vector.y = 1
    grasp.post_grasp_retreat.direction.vector.z = .0
    grasp.post_grasp_retreat.min_distance = 0.1
    grasp.post_grasp_retreat.desired_distance = 0.2 

    grasp.allowed_touch_objects.append('table_hb')

    my_move_group.move_group.pick("imu_hb", [grasp])


    # print(grasp)


def sensorPositioning():
    angle=rospy.get_param('/imu_angle')
    print('positioning imu with angle: {}'.format(angle))


def panelOpening():
    print('opening inspection panel')


def panelConverStorage():
    print('storing inspection panel')


def panelInspection():
    global secret_id
    print('inspecting panel')


def panelClosing():
    print('closing inspection panel')


def secretButton():
    secret_id=rospy.get_param('/secret_id')
    print('pressing button: {}'.format(secret_id))


def homePositioning():
    print('returning home position')
    gripperCommandClient("open")
    joint_vet=my_move_group.move_group.get_active_joints()
    for current_joint, home_value in enumerate(HOME_POSITION):
      joint_vet[current_joint]=grad_to_rad(home_value)
    my_move_group.go_to_joint_state(joint_vet)


#TODO: innested dictionary for function and PARAM name
objectives_to_actions={ '1':markersInspection,
                        '2':actuationOfButtons,
                        '3':sensorPickup,
                        '4':sensorPositioning,
                        '5':panelOpening,
                        '6':panelConverStorage,
                        '7':panelInspection,
                        '8':panelClosing,
                        '9':secretButton,
                        '10':homePositioning                      
                    }

def fakeController():
    global request_served, \
            group, \
            planning_frame ,\
            robot,\
            scene,\
            tf_buffer,tf_listener
    current_objective=0

    robot= RobotCommander()
    scene=PlanningSceneInterface()
    group_name="manipulator"
    group=MoveGroupCommander(group_name)
    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    # group_names = robot.get_group_names()
    # robot.get_current_state()
    # print(robot.get_current_state())

    tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(1))
    tf_listener=tf2_ros.TransformListener(tf_buffer,queue_size=None)
    
    #gripper command service
    rospy.wait_for_service(GRIPPER_SERVICE)

    while not rospy.core.is_shutdown() and not exit_request:
        

      try:
        # requested_objective=input('current objective: {}\n select: '.format(current_objective))
        requested_objective=rospy.get_param("/objective")#type: int
        
        if current_objective!=requested_objective:
          current_objective=requested_objective
          # objectives_to_actions[requested_objective]
          if current_objective==1:
              markersInspection()
          elif current_objective==2:
              actuationOfButtons()
          elif current_objective==3:
              sensorPickup()
          elif current_objective==4:
              sensorPositioning()
          elif current_objective==5:
              panelOpening()
          elif current_objective==6:
              panelConverStorage()
          elif current_objective==7:
              panelInspection()
          elif current_objective==8:
              panelClosing()
          elif current_objective==9:
              secretButton()
          elif current_objective==10:
              homePositioning()
          else:
              print('objectives are int, [1,10]')
          print('done')
        rospy.rostime.wallsleep(0.5)
      except KeyboardInterrupt or rospy.ROSInterruptException:
            rospy.signal_shutdown()

    
###########################################################

if __name__ == '__main__':
    
    node_name="controller"
    print('node name: {}\n arm control routine'.
        format(node_name))

    rospy.init_node(node_name,anonymous=False)
    
    moveit_commander.roscpp_initialize([])
    fakeController()

    try:
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInterruptException:
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))


"""
CURRENT TASK
set to zero once task is served; ask for confirmation if task is successfull

ADDITIONAL INPUT PARAMS
could delete once task is served

MOTION ROUTINES
could be coded somewhere and performed in order by a counter
incremento contatore se determinati sistemi di riferimento non sono creati prima della 
  conclusione della routine corrent

basic routine going in a certain position (eg: facing mid panel) and input param (ie: id)
  to perform a specific motion toward that specific, unfound, marker

PREDEFINED POSITINOS:
-ABSOLUTE: HOME, MID_PANEL, LEFT_PANEL,...
-RELATIVE: pre-approach position to perform some task
            +usefull working point/TPC for each task

GRIPPER_HB
-foreach gripper position change both "useful point" (TPC) and hitbox
bool update_gripper_collision_box(string new_position){
  /*Diversamente dalle altre funzioni, qui verrà fatto tutto rispetto al tool. Questo per mantenere solidale la box*/
  Affine3d T_tool_to_center_of_box;

  PoseStamped box_pose;
  float box_size[3];
  string box_name="gripper_box";
  box_pose.header.frame_id="ee_link";
  box_size[1]=0.03;
  if(new_position=="open"){

    T_tool_to_center_of_box.translation().x()=(0.096+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione

    box_size[0]=0.12;
    box_size[2]=0.13;

  }
  if(new_position=="semi_open"){

    T_tool_to_center_of_box.translation().x()=(0.128+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione



    box_size[0]=(0.128+0.025/2)*1.1;
    box_size[2]=0.1;

  }
  if(new_position=="semi_close"){

    T_tool_to_center_of_box.translation().x()=(0.132+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione




    box_size[0]=(0.132+0.025/2)*1.1;
    box_size[2]=0.09;

  }
  if(new_position=="close"){

    T_tool_to_center_of_box.translation().x()=(0.135+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione



    box_size[0]=(0.135+0.025/2)*1.1;
    box_size[2]=0.07;

  }

  Pose pose_center_of_box=homo_to_pose(T_tool_to_center_of_box);
  box_pose.pose=pose_center_of_box;

  collision_boxes[box_name].name=box_name;
  collision_boxes[box_name].pose=box_pose;
  collision_boxes[box_name].size[0]=box_size[0];
  collision_boxes[box_name].size[1]=box_size[1];
  collision_boxes[box_name].size[2]=box_size[2];
  remove_and_detach_box(collision_boxes["gripper_box"]);
  add_and_attach_box(collision_boxes["gripper_box"]);
}


MOVEIT

https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

https://ros-planning.github.io/moveit_tutorials/

CONVERSION FUNCITONS

void tf2quat_to_pose(tf2::Quaternion q,Pose *p)
{
  p->orientation.x=q.getX();
  p->orientation.y=q.getY();
  p->orientation.z=q.getZ();
  p->orientation.w=q.getW();
}

Pose homo_to_pose(Affine3d homo){

  tf::Pose pose_tf;
  Pose pose;
  tf::poseEigenToTF(homo,pose_tf);
  tf::poseTFToMsg(pose_tf,pose);
  return pose;

}

RPY POSE

void SetPoseOrientationRPY(Pose *p,int x0,int y0,int z0)
{
  tf2::Quaternion quat;
  quat.setRPY(grad_to_rad(x0),grad_to_rad(y0),grad_to_rad(z0));
  p->orientation.x=quat.getX();
  p->orientation.y=quat.getY();
  p->orientation.z=quat.getZ();
  p->orientation.w=quat.getW();
}


PLANNING

robot->setPlannerId("RRTConnectkConfigDefault");
robot->setPlanningTime(5);
robot->setNumPlanningAttempts(1000);

#NOTE: planning: success? -> move


LOAD STANDARD POSITIONS FROM FILE

string pkgpath = ros::package::getPath("arm_control");
string path_txt="/config/standard_positions.txt";
string path_total=pkgpath + path_txt;
THEN CYCLICALLY LOAD FROM FILE


CANCEL TRAJECTORY
  
  #PREFERRED
  MoveGroupInterface move_group(PLANNING_GROUP);
  robot=&move_group;
  robot->stop
  #ELSE 
  pub_traj_cancel = n.advertise<actionlib_msgs::GoalID>("/arm_controller/follow_joint_trajectory/cancel", 100);
                                                        "/move_group/cancel"
  actionlib_msgs::GoalID msg_traj_cancel;
  msg_traj_cancel.id="";
  pub_traj_cancel.publish(msg_traj_cancel);


############
  gazebo_model_state_pub = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
  pose_object_client = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");


PICK N PLACE
#NOTE: should consider future TCP when deciding pose for picking an object (or any other option)
# eg:   gripper's tip when pughing bottons, specific TCP for each specific aperture for each action
#NOTE: remember to add picked object hitbox to gripper
gazebo_msgs.msg import ModelState

void pick(string name_object,double distance_from_ee){
  Pose pose;
  pose.position.x = 0.0;
  pose.position.y = distance_from_ee+0.0819;
  pose.position.z = 0.0;
  pose.orientation.w = 1;

  gazebo_msgs::ModelState model_state;
  model_state.model_name = name_object;
  model_state.pose = pose;
  model_state.reference_frame =std::string("wrist_3_link");

  picked=true;
  while(picked==true){
  gazebo_model_state_pub.publish(model_state);

PUBLISER_TOPIC="/gazebo/set_model_state",



  gazebo_model_state_pub = rospy.Publisher(PUBLISHER_TOPIC,ModelState)
  REFERENCE_FRAME="wrist_3_link"
  model_state=ModelState(model_name=MODEL_NAME,
                        pose=Pose(position=POSITION,orientation=ORIENTATION)
                        reference_frame=REFERENCE_FRAME)
                        
  gazebo_model_state_pub.publish(model_state)
  
"""