#!/usr/bin/env python
import rospy, csv, time, socket, pickle, numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32MultiArray
from multiprocessing.connection import Listener, Client


# Setup TCP server
TCP_IP = '192.168.11.36'
TCP_PORT = 5007
BUFFER_SIZE = 4096
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Initialise lists
patch_number = 6
taxel_number = 16
tactile_list_v1 = Int32MultiArray()


def publisher():

    print('Publishing ROS topic!')

    while True:

        global tactile_list_v1
        #data_v1 = s.recv(BUFFER_SIZE)
        data_v1 = client.recv()

        if not data_v1:
            break

        elif data_v1:
            #tactile_list_v1.data = pickle.loads(data_v1)
            tactile_list_v1.data = data_v1
            pub.publish(tactile_list_v1)


if __name__== '__main__':

    try:
        pub = rospy.Publisher('tactile_reading_finger_1', Int32MultiArray, queue_size=10)
        rospy.init_node('tactile_reading_finger_1', anonymous=True)
        time.sleep(0.1)

        print('Connecting to server..')
        #s.connect((TCP_IP, TCP_PORT))
        client = Client((TCP_IP, TCP_PORT))

        print('Connected to server!')
        publisher()

    except rospy.ROSInterruptException:
                pass
