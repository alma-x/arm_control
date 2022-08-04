#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from arm_control.srv import GripperCommand,GripperCommandResponse
GRIPPER_TOPIC="/gripper_command"
GRIPPER_SERVICE='gripper_commands'

# gripper_pub=None

def gripperCommandServer(command_req):
    # global gripper_pub
    command=command_req.command
    print("received command: ",command)
    if command=="open" or \
    command=="semi_open" or \
    command=="semi_close" or \
    command=="close":
        gripper_pub=rospy.Publisher(GRIPPER_TOPIC,String,queue_size=5)
        gripper_msg=String()
        gripper_msg.data=command
        gripper_pub.publish(gripper_msg)
    else:
        print("wrong state requested!\n")
    # command_response=GripperCommandResponse()
    # command_response.done=True
    # return command_response
    return True

def gripperCommander():
    rospy.Service(GRIPPER_SERVICE, GripperCommand, gripperCommandServer)

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
