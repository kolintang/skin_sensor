#!/usr/bin/env python

import sys, struct, argparse, errno
import socket, csv, rospy
from std_msgs.msg import String,Int32,Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class CANSocket(object):
  FORMAT = "<IB3x8s"
  FD_FORMAT = "<IB3x64s"
  CAN_RAW_FD_FRAMES = 5
  num_taxel = 16 

  def __init__(self, interface=None):
    self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    if interface is not None:
      self.bind(interface)

  def bind(self, interface):
    self.sock.bind((interface,))
    self.sock.setsockopt(socket.SOL_CAN_RAW, self.CAN_RAW_FD_FRAMES, 1)

  def send(self, cob_id, data, flags=0):
    cob_id = cob_id | flags
    can_pkt = struct.pack(self.FORMAT, cob_id, len(data), data)
    self.sock.send(can_pkt)

  def recv(self, flags=0):
    can_pkt = self.sock.recv(72)

    if len(can_pkt) == 16:
      cob_id, length, data = struct.unpack(self.FORMAT, can_pkt)
    else:
      cob_id, length, data = struct.unpack(self.FD_FORMAT, can_pkt)

    cob_id &= socket.CAN_EFF_MASK
    return (cob_id, data[:length])


def format_data(data):
    return ' '.join([hex(byte)[2:] for byte in data])


def generate_bytes(hex_string):
    if len(hex_string) % 2 != 0:
      hex_string = "0" + hex_string

    int_array = []
    for i in range(0, len(hex_string), 2):
        int_array.append(int(hex_string[i:i+2], 16))
    return bytes(int_array)


def send_cmd(args):
    try:
      s = CANSocket(args.interface)
    except OSError as e:
      sys.stderr.write('Could not send on interface {0}\n'.format(args.interface))
      sys.exit(e.errno)

    try:
      cob_id = int(args.cob_id, 16)
    except ValueError:
      sys.stderr.write('Invalid cob-id {0}\n'.format(args.cob_id))
      sys.exit(errno.EINVAL)

    s.send(cob_id, generate_bytes(args.body), socket.CAN_EFF_FLAG if args.extended_id else 0)


def listen_cmd(args):
    try:
      s = CANSocket(args.interface)
    except OSError as e:
      sys.stderr.write('Could not listen on interface {0}\n'.format(args.interface))
      sys.exit(e.errno)
    print('Listening on {0} \n'.format(args.interface))
    cob_id, data = s.recv()

    # Initialising CSV file
    csv_write = False
    object_name = 'bottle_00.csv'
    if csv_write:
        csv_name = '../Datasets/' + object_name
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
                '2B4S3X','2B4S3Y','2B4S3Z','2B4S4X','2B4S4Y','2B4S4Z'))
    else:
        pass

    # How many skin sensor patch used
    patch_number = 3

    # Initialising Buffer
    row = [None]*48*patch_number
    x_axis = [None]*16*patch_number
    y_axis = [None]*16*patch_number
    z_axis = [None]*16*patch_number
    
    # Buffer for 16 nodes with 3 axis each
    tactle_arrary = [None]*8*16*patch_number

    # Expected sensor node address, 16 nodes per patch
    CAN_address = [0x720, 0x721, 0x722, 0x723,
                   0x724, 0x725, 0x726, 0x727,
                   0x728, 0x729, 0x72A, 0x72B,
                   0x72C, 0x72D, 0x72E, 0x72F,

                   0x730, 0x731, 0x732, 0x733,
                   0x734, 0x735, 0x736, 0x737,
                   0x738, 0x739, 0x73A, 0x73B,
                   0x73C, 0x73D, 0x73E, 0x73F,

                   0x740, 0x741, 0x742, 0x743,
                   0x744, 0x745, 0x746, 0x747,
                   0x748, 0x749, 0x74A, 0x74B,
                   0x74C, 0x74D, 0x74E, 0x74F]

    # Initialising ROS Topic/Node
    pub = rospy.Publisher('tactile_reading', Int32MultiArray, queue_size=0)
    rospy.init_node('tactile_sensor', anonymous=True)
    rate = rospy.Rate(120)

    while True:
        cob_id, data = s.recv() #There are 16 cob ID in a patch
        #print('COD ID: %s | %s %03x#%s' % (cob_id, args.interface, cob_id, format_data(data)))
        
        # Check if each nodes are updated at a instance
        if None in tactle_arrary:
            for i in range(16*patch_number):
                if(cob_id == CAN_address[i]):
                    for j in range(8):
                        tactle_arrary[i*8+j] = data[j]

                    x_axis[i] = tactle_arrary[i*8+1] << 8 | tactle_arrary[i*8+2]  
                    y_axis[i] = tactle_arrary[i*8+3] << 8 | tactle_arrary[i*8+4]
                    z_axis[i] = tactle_arrary[i*8+5] << 8 | tactle_arrary[i*8+6]

                    row[i*3] = x_axis[i]
                    row[i*3+1] = y_axis[i]
                    row[i*3+2] = z_axis[i]

        else:
            # Write to CSV at once
            if csv_write:
                print ('Fully updated.. writing to CSV\n')
                with open(csv_name, 'a') as csvfile:
                    writer = csv.writer(csvfile, lineterminator='\n')
                    writer.writerow(row)
            else:
                pass

            # Initialise publish list in ROS message type
            row_pub = Int32MultiArray()
            row_pub.data = row

            # Publish ROS message
            #rospy.loginfo(row_pub)
            pub.publish(row_pub)
            rate.sleep()

            # Reinitialising Buffer
            tactle_arrary = [None]*8*16*patch_number


def parse_args():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    send_parser = subparsers.add_parser('send', help='send a CAN packet')
    send_parser.add_argument('interface', type=str, help='interface name (e.g. vcan0)')
    send_parser.add_argument('cob_id', type=str, help='hexadecimal COB-ID (e.g. 10a)')
    send_parser.add_argument('body', type=str, nargs='?', default='',
      help='hexadecimal msg body up to 8 bytes long (e.g. 00af0142fe)')
    send_parser.add_argument('-e', '--extended-id', action='store_true', default=False,
      help='use extended (29 bit) COB-ID')
    send_parser.set_defaults(func=send_cmd)

    listen_parser = subparsers.add_parser('listen', help='listen for and print CAN packets')
    listen_parser.add_argument('interface', type=str, help='interface name (e.g. vcan0)')
    listen_parser.set_defaults(func=listen_cmd)
    return parser.parse_args()


def main():
    args = parse_args()
    args.func(args)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

