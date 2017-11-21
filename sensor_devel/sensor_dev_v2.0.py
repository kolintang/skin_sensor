
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
    print('Listening on {0} \n'.format(args.interface))
    cob_id, data = s.recv()
    csv_name = 'test_log_' + args.interface + '.csv'

    row = [None]*48
    x_axis = [None]*16
    y_axis = [None]*16
    z_axis = [None]*16
    
    tactle_arrary = [
            [None]*8, [None]*8, [None]*8, [None]*8,
            [None]*8, [None]*8, [None]*8, [None]*8,
            [None]*8, [None]*8, [None]*8, [None]*8,
            [None]*8, [None]*8, [None]*8, [None]*8] # 16 nodes with 3 axis

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
    
    CAN_address = [0x710, 0x712, 0x711, 0x713,
                   0x714, 0x716, 0x715, 0x717,
                   0x718, 0x71A, 0x719, 0x71B,
                   0x71C, 0x71E, 0x71D, 0x71F]
    while True:
        cob_id, data = s.recv() #There are 16 cob ID in a patch
        print('COD ID: %s | %s %03x#%s' % (cob_id, args.interface, cob_id, format_data(data)))

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

                print('Tactle Arrary %s: %s' % (j, tactle_arrary[i]))
                print('X-axis Arrary %s: %s' % (j, x_axis[i]))
                print('Y-axis Arrary %s: %s' % (j, y_axis[i]))
                print('Z-axis Arrary %s: %s' % (j, z_axis[i]))
                print('Sensor Patch Reading: %s' % (row))
                print()

        with open(csv_name, 'a') as csvfile:
            writer = csv.writer(csvfile, lineterminator='\n')
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
