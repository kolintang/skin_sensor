#!/usr/bin/env python
import rospy, csv, time, socket, pickle, numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32MultiArray


# Initialise csv path and dataset name
object_name = 'test'
trial_number = '00'
csv_name = '../datasets/' + object_name + '_' + trial_number + '.csv'


# Initialise lists
finger_numer = 1
patch_number = 5
taxel_number = 16
csv_header = []
tactile_list_1 = [None]*48*patch_number
#tactile_list_2 = [None]*48*patch_number
#tactile_list_3 = [None]*48*patch_number
#tactile_list_4 = [None]*48*patch_number
joint_list = [None]*16*patch_number
JT_List = [None]*64*patch_number


def csv_init():
    # Initialising taxel number header
    for i in range (finger_number):
        for j in range (patch_number):
            for k in range (taxel_number):
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


def callback_node_state(state):

    global node_state
    node_state = state.data


def callback_joint(joint):

    global joint_list
    global node_state

    if node_state == 1:
        # Write to new csv
        joint_list = joint.position
        write_csv()


def callback_tactile_1(tactile):
    tactile_list_1 = tactile.data


def callback_tactile_2(tactile):
    tactile_list_2 = tactile.data


def callback_tactile_3(tactile):
    tactile_list_3 = tactile.data


def callback_tactile_4(tactile):
    tactile_list_4 = tactile.data


def write_csv():

    if None in tactile_list_1
       #None in tactile_list_2 or
       #None in tactile_list_3 or
       #None in tactile_list_4 or
       or None in joint_list:
        pass

    else:
        print ('Fully updated.. publishing\n')
        JT_List = list(tactile_list_1) +
                  #list(tactile_list_2) +
                  #list(tactile_list_3) +
                  #list(tactile_list_4) +
                  list(joint_list)

        # Write to CSV at once
        with open(csv_name, 'a') as csvfile:
            writer = csv.writer(csvfile, lineterminator='\n')
            writer.writerow(JT_List)

        # Reinitialising Buffer
        tactile_list_1 = [None]*48*patch_number
        #tactile_list_2 = [None]*48*patch_number
        #tactile_list_3 = [None]*48*patch_number
        #tactile_list_4 = [None]*48*patch_number
        joint_list = [None]*16*patch_number


def listener():
    rospy.init_node('joint_tactile_subscriber', anonymous=True)
    rospy.Subscriber('allegroHand/node_states', Int32, callback_node_state)
    rospy.Subscriber('allegroHand_0/joint_states', JointState, callback_joint)
    rospy.Subscriber('tactile_reading_finger_1', Int32MultiArray, callback_tactile_1)
    #rospy.Subscriber('tactile_reading_finger_2', Int32MultiArray, callback_tactile_2)
    #rospy.Subscriber('tactile_reading_finger_3', Int32MultiArray, callback_tactile_3)
    #rospy.Subscriber('tactile_reading_finger_4', Int32MultiArray, callback_tactile_4)
    rospy.spin()


if __name__== '__main__':
    csv_init()
    listener()
