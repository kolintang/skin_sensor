#!/usr/bin/env python3

import logging
import time
import can

def simple_periodic_send(bus):
    """
    Sends a message every 20ms with no explicit timeout
    Sleeps for 2 seconds then stops the task.
    """
    print("Starting to send a message every 200ms for 2s")
    msg = can.Message(arbitration_id=0x123, data=[1, 2, 3, 4, 5, 6], extended_id=False)
    task = bus.send_periodic(msg, 0.20)
    assert isinstance(task, can.CyclicSendTaskABC)
    time.sleep(2)
    task.stop()
    print("stopped cyclic send")

def multiple_frame_send(bus)
    print("Starting to send multiple frames message")
    msg1 = can.Message(arbitration_id=0x123, data=[1, 2, 3, 4, 5, 6], extended_id=False)
    msg2 = can.Message(arbitration_id=0x321, data=[1, 2, 3, 4, 5, 6], extended_id=False)
    task = bus.send_periodic(msg, 0.20)
    pass



if __name__ == "__main__":

    reset_msg = can.Message(arbitration_id=0x00, data=[0, 0, 0, 0, 0, 0], extended_id=False)

    for interface in {'socketcan_native'}:
        print("Carrying out cyclic tests with {} interface".format(interface))
        can.rc['interface'] = interface
        channel = 'vcan0'
        bus = can.interface.Bus(channel=channel)

        bus.send(reset_msg)
        simple_periodic_send(bus)

    time.sleep(2)
