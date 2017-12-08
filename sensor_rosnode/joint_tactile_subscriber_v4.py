#!/usr/bin/env python
import rospy, csv, time, socket, pickle, numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32MultiArray


# Setup TCP server
TCP_IP = '192.168.11.11'
TCP_PORT = 5080
BUFFER_SIZE = 8192
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


# Initialise csv path and dataset name
object_name = 'test'
trial_number = '00'
csv_name = '../datasets/' + object_name + '_' + trial_number + '.csv'


# Initialise lists
finger_numer = 3
patch_number = 5
taxel_number = 16
csv_header = []
tactile_list_0 = [None]*48*patch_number
tactile_list_1 = [None]*48*patch_number
tactile_list_2 = [None]*48*patch_number
tactile_list_3 = [None]*48*patch_number
joint_list = [None]*16*patch_number
JT_List = [None]*64*patch_number


def csv_init():
    # Initialising taxel number header
    for i in range (finger_number)
        for j jn range (patch_number):
            for k jn range (taxel_number):
                csv_header.append(str(i)+'F'+str(j)+'B'+str(k)+'X')
                csv_header.append(str(i)+'F'+str(j)+'B'+str(k)+'Y')
                csv_header.append(str(i)+'F'+str(j)+'B'+str(k)+'Z')

    # Initialising finger & joint number header
    for k in range (4):
        for l in range (4):
            csv_header.append('F'+str(k)+'J'+str(l))

    # Initialising CSV header
    with open(csv_name, 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(csv_header)


def callback_joint(joint):
    joint_list = joint.position
    write_csv()


def callback_tactile_0(tactile):
    tactile_list_0 = tactile.data
    write_csv()


def callback_tactile_1(tactile):
    tactile_list_1 = tactile.data
    write_csv()


def callback_tactile_2(tactile):
    tactile_list_2 = tactile.data
    write_csv()


def callback_tactile_3(tactile):
    tactile_list_3 = tactile.data
    write_csv()


def write_csv():

    if None in joint_list or None in tactile_list_0
    or None in tactile_list_1 or None in tactile_list_2 or None in tactile_list_3:
        pass

    else:
        print ('Fully updated.. publishing\n')
        JT_List = list(tactile_list_0) + list(tactile_list_1) + list(tactile_list_2) + 
        list(tactile_list_3) + list(joint_list)

        # Write to CSV at once
        with open(csv_name, 'a') as csvfile:
            writer = csv.writer(csvfile, lineterminator='\n')
            writer.writerow(JT_List)

        # Reinitialising Buffer
        tactile_list_0 = [None]*48*patch_number
        tactile_list_1 = [None]*48*patch_number
        tactile_list_2 = [None]*48*patch_number
        tactile_list_3 = [None]*48*patch_number
        joint_list = [None]*16*patch_number 


def listener():
    rospy.init_node('joint_tactile_subscriber', anonymous=True)
    rospy.Subscriber('allegroHand_0/joint_states', JointState, callback_joint)
    rospy.Subscriber('tactile_reading_finger_0', Int32MultiArray, callback_tactile_0)
    rospy.Subscriber('tactile_reading_finger_1', Int32MultiArray, callback_tactile_1)
    rospy.Subscriber('tactile_reading_finger_2', Int32MultiArray, callback_tactile_2)
    rospy.Subscriber('tactile_reading_finger_3', Int32MultiArray, callback_tactile_3)
    rospy.spin()


if __name__== '__main__':
    csv_init()
    listener()
