# before first use type: pip install digi-xbee in command window
# it will install library that helps using xbee device
# this part of code should check if xbee library was installed
# and if wasn't this code should download it
from ctypes.wintypes import BYTE
from multiprocessing.spawn import prepare
import os
import string
from time import sleep
import numpy as np

from pyrsistent import b
try:
    import digi.xbee.devices as xb
except ModuleNotFoundError:
    # Error handling
    os.system("pip install digi-xbee")
    import digi.xbee.devices as xb


class Xbee:
    xbeePort = "COM3"
    xbBoudrate = "9600"

    def __init__(self, port=xbeePort, boudrate=xbBoudrate) -> None:
        self.xbeePort = port
        self.xbBoudrate = boudrate
        self.xbDevice = xb.XBeeDevice(self.xbeePort, self.xbBoudrate)
        self.xbDevice.open()
        self.xbDevice.add_data_received_callback(self.__callbackDataReceived)

    def __del__(self):
        self.xbDevice.del_data_received_callback(self.__callbackDataReceived)
        self.xbDevice.close()
        print("Connection closed")

    def send_msg_broadcast(self, msg: str) -> None:
        self.xbDevice.send_data_broadcast(msg)

    def send_msg_unicast(self, remote_adr: str, msg: str) -> None:
        remote = xb.RemoteXBeeDevice(
            self.xbDevice, xb.XBee64BitAddress.from_hex_string(remote_adr))
        self.xbDevice.send_data(remote, msg)

    def __callbackDataReceived(self, msg: xb.XBeeMessage):
        id = msg.remote_device.get_16bit_addr().get_lsb()
        decoded_msg = msg.data.decode('utf-8')
        print(f"Robot {id}: {decoded_msg}")

class xbee_frame():
    def __init__(self):
        self.msg_send = 0
        # how many bytes has frame (with header and checksum):
        self.LENGTH = 16
        self.full_msg = np.zeros(self.LENGTH, dtype=np.uint8)

        # set HEADER as 2 first bytes:
        self.full_msg[0] = 0x20
        self.full_msg[1] = 0x40

        # set ENDER as 2 last bytes:
        self.full_msg[self.LENGTH-2] = 0x50
        self.full_msg[self.LENGTH-1] = 0x60

        self.checksum = 0

    def _prepare_msg_(self, task_id=0, distance=0, task_time=0, arc_radius=0, rotation_angle=0):

        # --------------CREATE FRAME--------------
        # header 2 bytes 0x20 0x40
        # task_id 1 byte
        # distance to reach 2 bytes
        # time for complete task 3 bytes
        # radius of arc 2 bytes
        # rotation 2 bytes checksum
        # ender 2 bytes 0x50 0x60

        # put values into full.msg:
        self.full_msg[2] = task_id & 0xff
        self.full_msg[3] = distance & 0xff
        self.full_msg[4] = (distance >> 8) & 0xff
        self.full_msg[5] = task_time & 0xff
        self.full_msg[6] = (task_time >> 8) & 0xff
        self.full_msg[7] = (task_time >> 16) & 0xff
        self.full_msg[8] = arc_radius & 0xff
        self.full_msg[9] = (arc_radius >> 8) & 0xff
        self.full_msg[10] = rotation_angle & 0xff
        self.full_msg[11] = (rotation_angle >> 8) & 0xff

        # calculate checksum
        self.checksum = sum(self.full_msg[0:self.LENGTH-4])

        # put checksum into full.msg
        self.full_msg[12] = self.checksum & 0xff
        self.full_msg[13] = (self.checksum >> 8) & 0xff

        # create bitarray (xbee requirements):
        self.full_msg = bytearray(self.full_msg)

    def send_msg(self, task_id=0, distance=0, task_time=0, arc_radius=0, rotation_angle=0):

        self._prepare_msg_(task_id, distance, task_time,
                           arc_radius, rotation_angle)

        # for debugging:
        print(self.full_msg)


if __name__ == '__main__':
    frame = xbee_frame()
    frame.send_msg(task_id=10, distance=256, task_time=45,
                   arc_radius=10, rotation_angle=40)
    