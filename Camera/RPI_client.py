import socket
import json

class RPI_Communication_Client():
    def __init__(self,host="localhost",port=9999):
        self.host=host
        self.port = port
        self.rpi_client_socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buffer=""
        self.message_sended=False
        self.HEADER_SIZE=10

    def send_json(self,json_message):

        # local_ip=socket.gethostbyname(socket.gethostname())
        json_data = json.dumps(json_message, sort_keys=False, indent=2)
        json_data=f'{len(json_data):<{self.HEADER_SIZE}}'+json_data
        self._send_message(json_data)
        return json_data    

    def _send_message(self,data):
        self.message_sended=False
        self.rpi_client_socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rpi_client_socket.connect((self.host, self.port))
        self.rpi_client_socket.sendall(data.encode())
        self.rpi_client_socket.close()
        self.message_sended=True
        
    def _client_listen(self):
        print("Client listening")
        self.buffer = self.rpi_client_socket.recv(1024)
        print("Received: ",repr(self.buffer))


client=RPI_Communication_Client('192.168.235.120')

data={"obstacles": [
{
    "id": 0,
    "center": {
    "x": 250,
    "y": 150
    },
    "rotation": 0
},
{
    "id": 1,
    "center": {
    "x": 1000,
    "y": 100
    },
    "rotation": 0
},
{
    "id": 2,
    "center": {
    "x": 1328,
    "y": 976
    },
    "rotation": 45
}
]
}
client.send_json(data)
