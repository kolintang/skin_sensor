
import can
import socket
import struct
import sys

bustype = 'socketcan_native'
channel = 'vcan0'

sender_socket = createSocket()
bindSocket(sender_socket, 'vcan0')
sender_socket.send(build_can_frame(0x01, b'\x01\x02\x03'))
print("Sender sent a message.")
