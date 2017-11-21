
import sys
import socket
import argparse
import struct
import errno
import csv


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
    print('Listening on {0}'.format(args.interface))

    row = [None]*48
    x_axis = [None]*16
    y_axis = [None]*16
    z_axis = [None]*16
    tactle_arrary = [[]] # 16 nodes with 3 axis
    cob_id, data = s.recv()
    csv_name = 'test_log_' + args.interface + '_' + hex(cob_id)[2:] + '.csv'

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
    
    while True:
        cob_id, data = s.recv()
        print('%s %03x#%s' % (args.interface, cob_id, format_data(data)))
        print(cob_id)

        if cob_id == 544:
            for byte in data:
                tactle_arrary[0].append(byte)

            x_axis[0] = tactle_arrary[0][1] << 8 | tactle_arrary[0][2]  
            y_axis[0] = tactle_arrary[0][3] << 8 | tactle_arrary[0][4]
            z_axis[0] = tactle_arrary[0][5] << 8 | tactle_arrary[0][6]

            row[0, 1, 2] = [x_axis[0], y_axis[0], z_axis[0]]
            #row[0] = x_axis[0]
            #row[1] = y_axis[0]
            #row[2] = z_axis[0]

            print(tactle_arrary[0])
            print(x_axis, y_axis, z_axis)

            with open(csv_name, 'a') as csvfile:
                writer = csv.writer(csvfile, lineterminator='\n')
                writer.writerow(row)
                #writer.writerow([x_axis[0], y_axis[0], z_axis[0]])
            
        if cob_id == 545:
            for byte in data:
                tactle_arrary[1].append(byte)

            x_axis[1] = tactle_arrary[1][1] << 8 | tactle_arrary[1][2]  
            y_axis[1] = tactle_arrary[1][3] << 8 | tactle_arrary[1][4]
            z_axis[1] = tactle_arrary[1][5] << 8 | tactle_arrary[1][6]

            print(tactle_arrary[1])
            print(x_axis, y_axis, z_axis)

            with open(csv_name, 'a') as csvfile:
                writer = csv.writer(csvfile, lineterminator='\n')
                row[3, 4, 5] = [x_axis[1], y_axis[1], z_axis[1]]
                writer.writerow(row)


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
    main()
