# before first use type: pip install digi-xbee in command window
# it will install library that helps using xbee device
# this part of code should check if xbee library was installed
# and if wasn't this code should download it
import os
import string

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

    
xbee = Xbee()
# while True:
xbee.send_msg_unicast("0013A200415E861B","<500>")
xbee.send_msg_unicast("0013A200415E861B","<1000>")

print("wyslane")
    # print("odbieram dane")
