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
patch_number = 5
taxel_number = 16
tactile_header = []
tactile_list = [None]*48*patch_number
joint_list = [None]*16*patch_number
JT_List = [None]*64*patch_number


def csv_init():
    # Initialising taxel number header
    for i in range (patch_number):
        for j in range (taxel_number):
            tactile_header.append(str(i)+'B'+str(j)+'X')
            tactile_header.append(str(i)+'B'+str(j)+'Y')
            tactile_header.append(str(i)+'B'+str(j)+'Z')

    # Initialising finger & joint number header
    for k in range (4):
        for l in range (4):
            tactile_header.append('F'+str(k)+'J'+str(l))

    # Initialising CSV header
    with open(csv_name, 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(tactile_header)


def callback_joint(joint):
    global joint_list
    joint_list = joint.position
    write_csv()


def callback_tactile():
    global tactile_list
    print ("received data: %s \n" % (tactile_list))
    rospy.Subscriber('allegroHand_0/joint_states', JointState, callback_joint)


def write_csv():
    
    global patch_number
    global tactile_list
    global joint_list
    global JT_List

    if None in joint_list or None in tactile_list:
        #print ('Still has not been fully updated\n')
        pass

    else:
        print ('Fully updated.. publishing\n')
        JT_List = list(tactile_list) + list(joint_list)
        
        # Write to CSV at once
        with open(csv_name, 'a') as csvfile:
            writer = csv.writer(csvfile, lineterminator='\n')
            writer.writerow(JT_List)

        # Reinitialising Buffer
        tactile_list = [None]*48*patch_number
        joint_list = [None]*16*patch_number 


def listener():
    rospy.init_node('joint_tactile_subscriber', anonymous=True)
    time.sleep(0.1)
    s.connect((TCP_IP, TCP_PORT))

    while True:
        global tactile_list
        data = s.recv(BUFFER_SIZE)
        if not data: break
        tactile_list = pickle.loads(data)
        callback_tactile()


if __name__ == '__main__':
    csv_init()
    listener()
