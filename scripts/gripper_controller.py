#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from arm_control.srv import GripperCommand,GripperCommandResponse
from geometry_msgs.msg import TransformStamped
import tf2_ros

GRIPPER_TOPIC="/gripper_command"
GRIPPER_SERVICE='gripper_commands'
gripper_state='idle'
# gripper_pub=None

gripper_TCP_state={ 'idle': .1515,
                    'open': .1515,
                    'semi_open': .186,
                    'semi_close': .1925,
                    'close': .2
                    }


tf_broadcaster=None

def gripperCommandServer(command_req):
    global gripper_state
    command=command_req.command
    print("received command: ",command)
    if command=="open" or \
    command=="semi_open" or \
    command=="semi_close" or \
    command=="close":
        gripper_pub=rospy.Publisher(GRIPPER_TOPIC,String,queue_size=5)
        gripper_msg=String()
        gripper_msg.data=command
        if not gripper_state==command:
            changeTCP()
            gripper_state=command
        gripper_pub.publish(gripper_msg)
    else:
        print("wrong state requested!\n")
    return True

def changeTCP():
    current_TCP=gripper_TCP_state[gripper_state]
    #broadcast TCP point

    #TODO: is tf the best way to insert another planning point/group?
    # tf_stamped=TransformStamped()
    # tf_stamped.header.stamp=rospy.Time.now()
    # current_frame="gripper_tcp"
    # gripper_base_frame=''
    # tf_stamped.child_frame_id=current_frame
    # tf_stamped.header.frame_id=gripper_base_frame
    # tf_stamped.transform.translation.x=object_pose.position.x
    # tf_stamped.transform.translation.y=object_pose.position.y
    # tf_stamped.transform.translation.z=object_pose.position.z
    # tf_stamped.transform.rotation.x=object_pose.orientation.x
    # tf_stamped.transform.rotation.y=object_pose.orientation.y
    # tf_stamped.transform.rotation.z=object_pose.orientation.z
    # tf_stamped.transform.rotation.w=object_pose.orientation.w
    # try:
    #     tf_broadcaster.sendTransform(tf_stamped)
    # except (tf2_ros.LookupException, \
    #         tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     pass


def gripperCommander():
    rospy.Service(GRIPPER_SERVICE, GripperCommand, gripperCommandServer)

    tf_broadcaster=tf2_ros.StaticTransformBroadcaster()

if __name__=='__main__':
    node_name='gripper_controller'
    rospy.init_node(node_name,anonymous=False)
    print("gripper control node\navailable states:\nopen\nsemi_open\nsemi_close\nclose")
    # gripper_pub=rospy.Publisher(GRIPPER_TOPIC,String,queue_size=5)
    # gripper_msg=String()
    gripperCommander()
    try:
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInterruptException:
        rospy.signal_shutdown('Esc key pressed; Closing node: {}'.format(node_name))


    # while True:
    #     try:
    #         gripper_data=str(input("gripper state: "))
    #         if gripper_data=="open" or \
    #             gripper_data=="semi_open" or \
    #             gripper_data=="semi_close" or \
    #             gripper_data=="close":

    #             gripper_msg.data=gripper_data
    #             gripper_pub.publish(gripper_msg)
    #         else:
    #             print("wrong state requested!\n")
    #     except KeyboardInterrupt: break
