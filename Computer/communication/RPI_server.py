import socket
import string
import threading

#Narazie komunikacja działa w jedna strone, tylko client wysyla. W tym momencie
#wiecej nie jest potrzebne

class RPI_Communication_Server:
    def __init__(self,host:string="localhost",port:int=9999):
        self._host=host
        self._port=port
        self._rpi_server_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM) #socket - IPv4 ,TCP protocol
        self._rpi_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rpi_server_socket.bind((self._host,self._port))
        self._buffer=""
        self.is_rpi_connected=False
        self.message_received=False
        self._rpi_communication_thread=threading.Thread(target=self._server_listen)
        self._rpi_communication_thread.start()
        self._command=""
        

    def _server_listen(self):
        self._rpi_server_socket.listen()
        self.is_rpi_connected=True
        while True:
            print("\nListening for client..")
            conn,addr = self._rpi_server_socket.accept()
            self.message_received=False
            print("Connection address: ",addr)
            self._buffer=conn.recv(1024)
            self._buffer=self._buffer.decode()
            self.message_received=True
            conn.close()

    def get_buffer(self):
        return self._buffer

    def clear_buffer(self):
        self._buffer=""
    
    def set_command(self,command):
        self._command=command
    
    def clear_command(self):
        self._command=""

    def _send_command(self,command,socket):
        encoded_command=bytes(command,'utf-8')
        socket.sendall(encoded_command)


server = RPI_Communication_Server(host='192.168.235.95')
while True:
    if server.get_buffer()!="":
        print(server.get_buffer())
        server.set_command("wysyłam")
        server.clear_buffer()