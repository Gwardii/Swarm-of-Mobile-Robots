
import cv2
from flask import Flask, render_template, Response, send_from_directory
from multiprocessing import Process
from pynput import keyboard
from threading import Thread
import socket
import os

# to achive live stream from RPI it is needed to start some kind of website.
# Flask is easier framework than Django but still it is more than enought for this application.
# Stream is really pimitive: some scripts saves frames in folder and this script use this images on website.

# define main FLask object:
app = Flask(__name__, template_folder="templates")


# define all necessary routs:

# main address:
@app.route('/')
def index():
    # render main html file as home page:
    return render_template('stream.html')


# route for favicon:
@app.route('/favicon.ico')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'),
                               'favicon.ico', mimetype='image/vnd.microsoft.icon')


# define main/video_feed route for stream:
@app.route('/video_feed')
def video_feed():
    return Response(stream_main_fun(), mimetype='multipart/x-mixed-replace; boundary=frame')


# main function for streaming:
def stream_main_fun():
    while True:
        # get camera frame from folder:
        frame = cv2.imread(
            './Camera/stream_tests/flask_version/stream/stream.jpg')

        if frame is not None:

            # define quality of stream and code img as bytes:
            frame = cv2.imencode('.JPEG', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[
                1].tobytes()

            # yield is like return but without ending loop:
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


def stream_start():
    # start stream (you can change port):

    # For RPI it is necessary to do this in this way :(
    # create socket connection and next read IP and close conection:

    con = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    con.connect(("8.8.8.8", 80))
    print(con.getsockname()[0])
    app.run(debug=True, host=con.getsockname()[0], port='9999', threaded=True)
    con.close()


if __name__ == "__main__":
    stream_start()
