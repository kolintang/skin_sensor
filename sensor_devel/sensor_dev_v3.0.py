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
    object_name = 'bottle_00.csv'
    csv_name = '../Datasets/' + object_name

    # Initialising Buffer
    row = [None]*48
    x_axis = [None]*16
    y_axis = [None]*16
    z_axis = [None]*16
    
    # Buffer for 16 nodes with 3 axis each
    tactle_arrary = [[None]*8, [None]*8, [None]*8, [None]*8,
                     [None]*8, [None]*8, [None]*8, [None]*8,
                     [None]*8, [None]*8, [None]*8, [None]*8,
                     [None]*8, [None]*8, [None]*8, [None]*8]

    # Initialising CSV file
    with open(csv_name, 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow((
            'B1S1X','B1S1Y','B1S1Z','B1S2X','B1S2Y','B1S2Z',
            'B1S3X','B1S3Y','B1S3Z','B1S4X','B1S4Y','B1S4Z',
            'B2S1X','B2S1Y','B2S1Z','B2S2X','B2S2Y','B2S2Z',
            'B2S3X','B2S3Y','B2S3Z','B2S4X','B2S4Y','B2S4Z',
            'B3S1X','B3S1Y','B3S1Z','B3S2X','B3S2Y','B3S2Z',
            'B3S3X','B3S3Y','B3S3Z','B3S4X','B3S4Y','B3S4Z',
            'B4S1X','B4S1Y','B4S1Z','B4S2X','B4S2Y','B4S2Z',
            'B4S3X','B4S3Y','B4S3Z','B4S4X','B4S4Y','B4S4Z'))
    
    # Expected sensor node address, 16 nodes per patch
    CAN_address = [0x740, 0x742, 0x741, 0x743,
                   0x744, 0x746, 0x745, 0x747,
                   0x748, 0x74A, 0x749, 0x74B,
                   0x74C, 0x74E, 0x74D, 0x74F]

    # Initialising ROS Topic/Node
    pub = rospy.Publisher('tactile_reading', Int32MultiArray, queue_size=10)
    rospy.init_node('tactile_sensor', anonymous=True)
    rate = rospy.Rate(300) # 10hz

    while True:

        cob_id, data = s.recv() #There are 16 cob ID in a patch
        print('COD ID: %s | %s %03x#%s' % (cob_id, args.interface, cob_id, format_data(data)))
        
        # Check if each nodes are updated at a instance
        if None in tactle_arrary[0] or None in tactle_arrary[1] or \
            None in tactle_arrary[2]  or None in tactle_arrary[3]  or \
            None in tactle_arrary[4]  or None in tactle_arrary[5]  or \
            None in tactle_arrary[6]  or None in tactle_arrary[7]  or \
            None in tactle_arrary[8]  or None in tactle_arrary[9]  or \
            None in tactle_arrary[10] or None in tactle_arrary[11] or \
            None in tactle_arrary[12] or None in tactle_arrary[13] or \
            None in tactle_arrary[14] or None in tactle_arrary[15]:
            print ('Still has not been fully updated')

            for i in range(16):
                if(cob_id == CAN_address[i]):
                    for j in range(8):
                        tactle_arrary[i][j] = data[j]

                    x_axis[i] = tactle_arrary[i][1] << 8 | tactle_arrary[i][2]  
                    y_axis[i] = tactle_arrary[i][3] << 8 | tactle_arrary[i][4]
                    z_axis[i] = tactle_arrary[i][5] << 8 | tactle_arrary[i][6]

                    row[i*3] = x_axis[i]
                    row[i*3+1] = y_axis[i]
                    row[i*3+2] = z_axis[i]

                    #print('Tactle Arrary %s: %s' % (j, tactle_arrary[i]))
                    #print('X-axis Arrary %s: %s' % (j, x_axis[i]))
                    #print('Y-axis Arrary %s: %s' % (j, y_axis[i]))
                    #print('Z-axis Arrary %s: %s' % (j, z_axis[i]))
                    #print('Sensor Patch Reading: %s' % (row))
                    print()

        else:
            # Write to CSV at once
            print ('Fully updated.. writing to CSV\n')
            with open(csv_name, 'a') as csvfile:
                writer = csv.writer(csvfile, lineterminator='\n')
                writer.writerow(row)

            # Publish ROS message
            #row = "Sensor Message: %s" % (str(row))
            row_pub = Int32MultiArray()
            row_pub.data = row
            rospy.loginfo(row_pub)
            pub.publish(row_pub)
            rate.sleep()

            # Reinitialising Buffer
            tactle_arrary = [[None]*8, [None]*8, [None]*8, [None]*8,
                             [None]*8, [None]*8, [None]*8, [None]*8,
                             [None]*8, [None]*8, [None]*8, [None]*8,
                             [None]*8, [None]*8, [None]*8, [None]*8]


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


def talker():
    while not rospy.is_shutdown():
        main()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

