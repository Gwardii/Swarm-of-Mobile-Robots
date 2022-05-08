# before first use type: pip install digi-xbee in command window
# it will install library that helps using xbee device
# this part of code should check if xbee library was installed
# and if wasn't this code should download it
import os
import string
import time
import struct

from pyrsistent import b
try:
    import digi.xbee.devices as xb
except ModuleNotFoundError:
    # Error handling
    os.system("pip install digi-xbee")
    import digi.xbee.devices as xb

class Xbee:
    xbeePort="COM3"
    xbBoudrate="9600"

    def __init__(self,port=xbeePort,boudrate=xbBoudrate) -> None:
        self.xbeePort=port
        self.xbBoudrate=boudrate
        self.xbDevice=xb.XBeeDevice(self.xbeePort,self.xbBoudrate)
        self.xbDevice.open()
        self.xbDevice.add_data_received_callback(self.__callbackDataReceived)
        
    def __del__(self):
        self.xbDevice.del_data_received_callback(self.__callbackDataReceived)
        self.xbDevice.close()
        print("Connection closed")

    def send_msg_broadcast(self,msg:str)->None:
        self.xbDevice.send_data_broadcast(msg)

    def send_msg_unicast(self,remote_adr:str,msg:str)->None:
        remote=xb.RemoteXBeeDevice(self.xbDevice,xb.XBee64BitAddress.from_hex_string(remote_adr))
        self.xbDevice.send_data(remote,msg)

    def __callbackDataReceived(self,msg:xb.XBeeMessage):
        id = msg.remote_device.get_16bit_addr().get_lsb()
        decoded_msg=msg.data.decode('utf-8')
        print(f"Robot {id}: {decoded_msg}")

    
# xbee = Xbee()
# tic=time.time()
# while( (time.time()-tic)<5):
#     xbee.send_msg_unicast("0013A200415E861B",)
# print("wyslane")
#     # print("odbieram dane")

import numpy as np

class xbee_frame():
    def __init__(self):
        self.msg_send = 0
        # how many bytes has frame (with header and checksum):
        self.LENGTH = 16
        self.full_msg = np.zeros(self.LENGTH, dtype=bytes)

        # set HEADER as 2 first bytes:
        self.full_msg[0] = (0x20).to_bytes(1, byteorder='big')
        self.full_msg[1] = (0x40).to_bytes(1, byteorder='big')

        # set ENDER as 2 last bytes:
        self.full_msg[self.LENGTH-2] = (0x50).to_bytes(1, byteorder='big')
        self.full_msg[self.LENGTH-1] = (0x60).to_bytes(1, byteorder='big')

        self.checksum = 0

    def _prepare_msg_(self, task_id=0, distance=0, task_time=0, arc_radius=0, rotation_angle=0):

        print(self.full_msg)
        print(distance.to_bytes(2, byteorder='big'))
        print(sum(distance.to_bytes(2, byteorder='big')))
        # put values into full.msg:
        self.full_msg[2] = task_id.to_bytes(1, byteorder='big')
        self.full_msg[3:5] = distance.to_bytes(2, byteorder='big')
        self.full_msg[5:8] = task_time.to_bytes(3, byteorder='big')
        self.full_msg[8:10] = arc_radius.to_bytes(2, byteorder='big')
        self.full_msg[10:12] = rotation_angle.to_bytes(2, byteorder='big')
        print(self.full_msg)

        # calculate checksum
        self.checksum = sum(self.full_msg[0:self.LENGTH-4])

        # put checksum into full.msg
        self.full_msg[12:14] = self.checksum.to_bytes(2, byteorder='big')

    def send_msg(self, task_id=0, distance=0, task_time=0, arc_radius=0, rotation_angle=0):

        self._prepare_msg_(task_id, distance, task_time,
                           arc_radius, rotation_angle)
        print(self.full_msg)
