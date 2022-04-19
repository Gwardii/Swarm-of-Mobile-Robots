import socket
import json

class RPI_Communication_Client():
    def __init__(self, host = "localhost", port = 9999):
        self.host = host
        self.port = port
        self.rpi_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buffer = ""
        self.message_sended=False
        self.buffer_size = 30
        self.HEADER_SIZE = 10

    def send_json(self,json_message):

        # local_ip=socket.gethostbyname(socket.gethostname())
        json_data = json.dumps(json_message, sort_keys=False, indent=2)
        print(len(json_data))
        json_data = f'{len(json_data):<{self.HEADER_SIZE}}' + json_data
        self._send_message(json_data)
        return json_data    

    def _send_message(self, data):

        self.message_sended = False
        self.rpi_client_socket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rpi_client_socket.connect((self.host, self.port))
        self.rpi_client_socket.sendall(data.encode())
        self.rpi_client_socket.close()
        self.message_sended = True
        
    def _client_listen(self):

        self.rpi_client_socket.connect((self.host, self.port))

        print("Client listening")
        full_msg = ""
        self._buffer = self.rpi_client_socket.recv(self.buffer_size)
        
        print(f"new message lenght: {self._buffer[:self.HEADER_SIZE].decode('utf-8')}")
        msglen = int(self._buffer[:self.HEADER_SIZE])
    
        while (len(full_msg) - self.HEADER_SIZE) < msglen:
            full_msg += self._buffer.decode("utf-8") 
            self._buffer = self.rpi_client_socket.recv(self.buffer_size)
            
        print("full msg recvd")
        self.full_msg = full_msg[self.HEADER_SIZE:]
        full_msg = ''
        self._buffer = ''
        self.message_received = True
        self.rpi_client_socket.close()


# client=RPI_Communication_Client(socket.gethostname())

# data={"obstacles": [
# {
#     "id": 0,
#     "center": {
#     "x": 250,
#     "y": 150
#     },
#     "rotation": 0
# },
# {
#     "id": 1,
#     "center": {
#     "x": 1000,
#     "y": 100
#     },
#     "rotation": 0
# },
# {
#     "id": 2,
#     "center": {
#     "x": 1328,
#     "y": 976
#     },
#     "rotation": 45
# }
# ]
# }
# client.send_json(data)
