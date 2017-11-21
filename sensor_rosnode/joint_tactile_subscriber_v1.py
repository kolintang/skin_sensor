#!/usr/bin/env python
import rospy, csv, time
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32MultiArray

# Initialising CSV file
csv_name = './CSV.csv'
with open(csv_name, 'w') as csvfile:
    writer = csv.writer(csvfile, lineterminator='\n')
    writer.writerow((
        '0B1S1X','0B1S1Y','0B1S1Z','0B1S2X','0B1S2Y','0B1S2Z',
        '0B1S3X','0B1S3Y','0B1S3Z','0B1S4X','0B1S4Y','0B1S4Z',
        '0B2S1X','0B2S1Y','0B2S1Z','0B2S2X','0B2S2Y','0B2S2Z',
        '0B2S3X','0B2S3Y','0B2S3Z','0B2S4X','0B2S4Y','0B2S4Z',
        '0B3S1X','0B3S1Y','0B3S1Z','0B3S2X','0B3S2Y','0B3S2Z',
        '0B3S3X','0B3S3Y','0B3S3Z','0B3S4X','0B3S4Y','0B3S4Z',
        '0B4S1X','0B4S1Y','0B4S1Z','0B4S2X','0B4S2Y','0B4S2Z',
        '0B4S3X','0B4S3Y','0B4S3Z','0B4S4X','0B4S4Y','0B4S4Z',

        '1B1S1X','1B1S1Y','1B1S1Z','1B1S2X','1B1S2Y','1B1S2Z',
        '1B1S3X','1B1S3Y','1B1S3Z','1B1S4X','1B1S4Y','1B1S4Z',
        '1B2S1X','1B2S1Y','1B2S1Z','1B2S2X','1B2S2Y','1B2S2Z',
        '1B2S3X','1B2S3Y','1B2S3Z','1B2S4X','1B2S4Y','1B2S4Z',
        '1B3S1X','1B3S1Y','1B3S1Z','1B3S2X','1B3S2Y','1B3S2Z',
        '1B3S3X','1B3S3Y','1B3S3Z','1B3S4X','1B3S4Y','1B3S4Z',
        '1B4S1X','1B4S1Y','1B4S1Z','1B4S2X','1B4S2Y','1B4S2Z',
        '1B4S3X','1B4S3Y','1B4S3Z','1B4S4X','1B4S4Y','1B4S4Z',
        
        '2B1S1X','2B1S1Y','2B1S1Z','2B1S2X','2B1S2Y','2B1S2Z',
        '2B1S3X','2B1S3Y','2B1S3Z','2B1S4X','2B1S4Y','2B1S4Z',
        '2B2S1X','2B2S1Y','2B2S1Z','2B2S2X','2B2S2Y','2B2S2Z',
        '2B2S3X','2B2S3Y','2B2S3Z','2B2S4X','2B2S4Y','2B2S4Z',
        '2B3S1X','2B3S1Y','2B3S1Z','2B3S2X','2B3S2Y','2B3S2Z',
        '2B3S3X','2B3S3Y','2B3S3Z','2B3S4X','2B3S4Y','2B3S4Z',
        '2B4S1X','2B4S1Y','2B4S1Z','2B4S2X','2B4S2Y','2B4S2Z',
        '2B4S3X','2B4S3Y','2B4S3Z','2B4S4X','2B4S4Y','2B4S4Z',

        'F0J0','F0J1','F0J2','F0J3',
        'F1J0','F1J1','F1J2','F1J3',
        'F2J0','F2J1','F2J2','F2J3',
        'F3J0','F3J1','F3J2','F3J3'))


def callback_joint(joint):
    # Write to joint list at once
    global joint_list
    joint_list = joint.position
    write_csv()


def callback_tactile(tactile):
    # Write to tactile list at once
    global tactile_list
    tactile_list = tactile.data
    write_csv()


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
        time.sleep(0) # for avoiding unsynced ros callback

        # Reinitialising Buffer
        tactile_list = [None]*48*patch_number
        joint_list = [None]*16*patch_number 


def listener():
    rospy.init_node('joint_tactile_subscriber', anonymous=True)
    rospy.Subscriber('allegroHand_0/joint_states', JointState, callback_joint)
    rospy.Subscriber('tactile_reading', Int32MultiArray, callback_tactile)
    rospy.spin()


if __name__ == '__main__':
    # Initialise joint/tactile list
    patch_number = 3
    tactile_list = [None]*48*patch_number
    joint_list = [None]*16*patch_number 
    JT_List = [None]*64*patch_number
    listener()
