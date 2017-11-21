import time
import can

bustype = 'socketcan_native'
channel = 'can0'

def command_loop(switch):
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    msg1 = can.Message(arbitration_id=0x202, data=[switch, 0], extended_id=False)
    msg2 = can.Message(arbitration_id=0x203, data=[switch, 0], extended_id=False)
    msg3 = can.Message(arbitration_id=0x204, data=[switch, 0], extended_id=False)

    bus.send(msg1)
    time.sleep(0.1)
    bus.send(msg2)
    time.sleep(0.1)
    bus.send(msg3)
    time.sleep(0.1)

while True:
    command_loop(7)
