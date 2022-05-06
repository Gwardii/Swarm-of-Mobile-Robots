import socket
import cv2
from threading import Thread
import numpy as np

# So this is script to transfer video with socket:


class Video_Stream_server():
    # create a server for video streaming
    def __init__(self, IP=socket.gethostname(), port=1234) -> None:

        # setup server
        self.ADDRESS = (IP, port)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(self.ADDRESS)
        print(f"SERVER adress: {self.ADDRESS}\n\r")

        # initialize the flag used to indicate if the thread should
        # be stopped
        self.stopped = False
        self.frame = None

    def _server_listen_(self):
        # start listening for new connection:
        self.server.listen()

        while True:
            # if thread stopped:
            if self.stopped:
                return

            # accept new connetions:
            clientsocket, address = self.server.accept()

            # receive data (frames):
            data, _ = clientsocket.recvfrom(921600)
            receive_data = np.frombuffer(data, dtype='uint8')
            self.frame = cv2.imdecode(receive_data, 1)
            cv2.imshow('img', self.frame)
            cv2.waitKey(1)

    def start(self):
        # start separated thread for server:
        self._server_thread = Thread(target=self._server_listen_)
        self._server_thread.start()

    def stop(self):

        self.stopped = True

    def Is_alive(self):
        # check if thread is working:
        return self._server_thread.is_alive()


class Video_Stream_client():
    # create client for video streaming
    def __init__(self, IP=socket.gethostname(), port=1234) -> None:

        # setup Adress of server
        self.ADDRESS = (IP, port)

    def stream_frame(self, frame):

        # connect to server:
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect(self.ADDRESS)

        # send frame:
        _, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 30])
        self.client.sendto(data, self.ADDRESS)

        # disconnect with server:
        self.client.close()


# # test everthing:
# vserver = Video_Stream_server()
# vserver.start()
# vclient = Video_Stream_client()
# cap = cv2.VideoCapture(0)

# while True:
#     _, frame = cap.read()
#     vclient.stream_frame(frame)


#     if not vserver.Is_alive():
#         break
