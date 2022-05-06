
import cv2
from flask import Flask, render_template, Response, send_from_directory
from multiprocessing import Process
from pynput import keyboard
from threading import Thread
import socket
import os


app = Flask(__name__, template_folder="templates")


@app.route('/')
def index():
    # rendering webpage
    return render_template('stream.html')


@app.route('/favicon.ico')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'),
                               'favicon.ico', mimetype='image/vnd.microsoft.icon')


def stream_main_fun():

    while True:
        # get camera frame

        frame = cv2.imread(
            './Camera/stream_tests/flask_version/stream/stream.jpg')

        if frame is not None:

            frame = cv2.imencode('.JPEG', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[
                1].tobytes()
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(stream_main_fun(), mimetype='multipart/x-mixed-replace; boundary=frame')


def stream_start():
    # defining server ip address and port
    con = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    con.connect(("8.8.8.8", 80))
    print(con.getsockname()[0])
    app.run(debug=True, host=con.getsockname()[0], port='9999', threaded=True)
    con.close()


def on_press(key):
    if key == keyboard.KeyCode.from_char('q'):
        return False
    return True


stream_start()
