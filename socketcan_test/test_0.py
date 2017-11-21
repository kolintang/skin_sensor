import socket
import struct
import sys

# CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
can_frame_fmt = "=IB3x8s"

def build_can_frame(can_id, data):
        can_dlc = len(data)
        data = data.ljust(8, b'\x00')
        return struct.pack(can_frame_fmt, can_id, can_dlc, data)

if len(sys.argv) != 2:
        print('Provide CAN device name (can0, slcan0 etc.)')
        sys.exit(0)

# create a raw socket and bind it to the given CAN interface
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((sys.argv[1],))
s.send(build_can_frame(0x204, b'\x01\x02\x03'))

