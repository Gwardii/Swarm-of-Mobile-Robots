import socket
import string
import threading
import json

#Narazie komunikacja działa w jedna strone, tylko client wysyla. W tym momencie
#wiecej nie jest potrzebne

class RPI_Communication_Server:
    def __init__(self,host = "localhost", port=9999):
        self._host=host
        self._port=port
        self._rpi_server_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM) #socket - IPv4 ,TCP protocol
        self._rpi_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rpi_server_socket.bind((self._host,self._port))
        self._buffer = ""
        self.buffer_size = 30
        self.is_rpi_connected=False
        self.message_received=False
        self._rpi_communication_thread=threading.Thread(target=self._server_listen)
        self._rpi_communication_thread.start()
        self._command = ""
        self.HEADER_SIZE = 10
        self.full_msg = ''
        

    def _server_listen(self):
        self._rpi_server_socket.listen()
        self.is_rpi_connected = True
        while True:
            
            full_msg=""
            print("\nListening for client..")
            conn, addr = self._rpi_server_socket.accept()
            self.message_received = False
            print("Connection address: ", addr)
            
            self._buffer = conn.recv(self.buffer_size)
            
            print(f"new message lenght: {self._buffer[:self.HEADER_SIZE].decode('utf-8')}")
            msglen = int(self._buffer[:self.HEADER_SIZE])
           
            while (len(full_msg) - self.HEADER_SIZE) < msglen:
                full_msg += self._buffer.decode("utf-8") 
                self._buffer = conn.recv(self.buffer_size)
                
            print("full msg recvd")
            self.full_msg = full_msg[self.HEADER_SIZE:]
            full_msg = ''
            self._buffer = ''
            self.message_received=True
            conn.close()
            

    def get_msg(self):
        return self.full_msg

    def clear_last_msg(self):
        self.full_msg = ""

    def clear_buffer(self):
        self._buffer = ""
    
    def set_command(self,command):
        self._command=command
    
    def clear_command(self):
        self._command = ""

    def _send_command(self,command,socket):
        encoded_command=bytes(command,'utf-8')
        socket.sendall(encoded_command)


# server = RPI_Communication_Server(socket.gethostname())
# while True:
#     if server.get_msg()!="":
        
#         data = json.loads(str(server.get_msg()))

#         if ('obstacles' in data.keys()):

#             with open("./Computer/resources/obstacles.json",'w') as f:
#                 json.dump(data, f, indent = 2)
#                 f.close()

#         elif ('robots' in data.keys()):

#             with open("./Computer/resources/robots.json",'w') as f:
#                 json.dump(data, f, indent = 2)
#                 f.close()

#         elif ('area' in data.keys()):

#             with open("./Computer/resources/area.json",'w') as f:
#                 json.dump(data, f, indent = 2)
#                 f.close()

#         else:
#              with open("./Computer/resources/dump_data.txt",'w') as f:
#                 json.dump(data, f, indent = 2)
#                 f.close()

#         server.clear_buffer()
#         server.clear_last_msg()