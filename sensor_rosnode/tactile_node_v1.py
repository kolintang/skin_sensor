#!/usr/bin/env python
import rospy, csv, time, socket, pickle, numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32MultiArray


# Setup TCP server
TCP_IP = '192.168.11.11'
TCP_PORT = 5007
BUFFER_SIZE = 8192
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


# Initialise lists
patch_number = 5
taxel_number = 16
tactile_header = []
tactile_list = [None]*48*patch_number
joint_list = [None]*16*patch_number
JT_List = [None]*64*patch_number


def callback_joint(joint):
    global joint_list
    joint_list = joint.position


def callback_tactile():
    global tactile_list
    print ("received data: %s \n" % (tactile_list))
    rospy.Subscriber('allegroHand_0/joint_states', JointState, callback_joint)


def listener():
    rospy.init_node('tactile_sensor_finger_1', anonymous=True)
    time.sleep(0.1)
    s.connect((TCP_IP, TCP_PORT))

    while True:
        global tactile_list
        data = s.recv(BUFFER_SIZE)
        if not data: break
        tactile_list = pickle.loads(data)
        callback_tactile()

if __name__== '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
                pass
