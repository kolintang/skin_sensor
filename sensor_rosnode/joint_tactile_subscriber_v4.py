#!/usr/bin/env python
import rospy, csv, pickle, numpy
import os, sys, time, datetime, thread, socket
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32, Int32MultiArray


# Initialise csv path and dataset name
object_name = 'baseball'
trial_number = 17
csv_name = '../datasets/' + object_name + '_' + str('%02d' % trial_number) + '.csv'


# Initialise lists
node_state = 0
finger_number = 4
patch_number = 13
taxel_number = 16
csv_header = []
tactile_list_1 = [None]*48*patch_number
tactile_list_2 = [None]*48*patch_number
tactile_list_3 = [None]*48*patch_number
tactile_list_4 = [None]*48*patch_number
joint_list = [None]*16*patch_number
combined = [None]*64*patch_number


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

    # Create first CSV and add header row
    with open(csv_name, 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(csv_header)

    print('Ready to write to file: %02d' % trial_number)

    while True:

        global trial_number
        global csv_name
        input('Press entre to create next CSV file')

        # Check existing CSV, and name a new one
        if os.path.exists(csv_name):
            trial_number += 1
            csv_name = '../datasets/' + object_name + '_' + str('%02d' % trial_number) + '.csv'

            # Create CSV and add header row
            with open(csv_name, 'w') as csvfile:
                writer = csv.writer(csvfile, lineterminator='\n')
                writer.writerow(csv_header)

        print('\nReady to write to file: %02d' % trial_number)


def callback_node_state(state):
    global node_state
    node_state = state.data


def callback_joint(joint):
    global joint_list
    joint_list = joint.position


def callback_tactile_1(tactile):
    global tactile_list_1
    tactile_list_1 = tactile.data


def callback_tactile_2(tactile):
    global tactile_list_2
    tactile_list_2 = tactile.data


def callback_tactile_3(tactile):
    global tactile_list_3
    tactile_list_3 = tactile.data


def callback_tactile_4(tactile):
    global tactile_list_4
    tactile_list_4 = tactile.data


def write_csv():

    global joint_list
    global tactile_list_1
    global tactile_list_2
    global tactile_list_3
    global tactile_list_4

    if None in tactile_list_1 or \
       None in tactile_list_2 or \
       None in joint_list:

       #None in tactile_list_3 or
       #None in tactile_list_4 or

        pass

    else:
        #print('Fully updated.. publishing\n')
        combined = list(tactile_list_1) + \
                   list(tactile_list_2) + \
                   list(joint_list)

                   #list(tactile_list_3) + \
                   #list(tactile_list_4) + \

        # Write to CSV at once
        with open(csv_name, 'a') as csvfile:
            writer = csv.writer(csvfile, lineterminator='\n')
            writer.writerow(combined)

        # Reinitialising Buffer
        joint_list = [None]*16*patch_number
        tactile_list_1 = [None]*48*patch_number
        tactile_list_2 = [None]*48*patch_number
        #tactile_list_3 = [None]*48*patch_number
        #tactile_list_4 = [None]*48*patch_number


def listener():

    print('\nObject Name: %s' % (object_name))
    print('Trial Number: %s' % (trial_number))
    print('Patch Number: %s' % (patch_number))

    print('\nSetting up ROS subscriber node..')
    rospy.init_node('joint_tactile_subscriber', anonymous=True)
    rospy.Subscriber('allegroHand/node_states', Int32, callback_node_state)
    rospy.Subscriber('allegroHand_0/joint_states', JointState, callback_joint)
    rospy.Subscriber('tactile_reading_finger_1', Int32MultiArray, callback_tactile_1)
    rospy.Subscriber('tactile_reading_finger_2', Int32MultiArray, callback_tactile_2)
    #rospy.Subscriber('tactile_reading_finger_3', Int32MultiArray, callback_tactile_3)
    #rospy.Subscriber('tactile_reading_finger_4', Int32MultiArray, callback_tactile_4)
    time.sleep(0.1)
    print('ROS subscriber setup!\n')

    thread.start_new_thread(csv_init, ())

    try:
        while not rospy.is_shutdown():
            global node_state
            if node_state == 1:
                #print('%s: Writing to CSV..' % (datetime.datetime.now()))
                write_csv()

            else:
                pass
                #print('%s: Waiting for record commands..' % (datetime.datetime.now()))

    except KeyboardInterrupt:
       print('Stop')
       sys.exit(1)


if __name__== '__main__':
    listener()
