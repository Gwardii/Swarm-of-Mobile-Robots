from multiprocessing.dummy import freeze_support
import cv2
from flask import Flask, render_template, Response
from multiprocessing import Process
from pynput import keyboard
from threading import Thread

app = Flask(__name__, template_folder = "templates")

@app.route('/')
def index():
    # rendering webpage
    return render_template('stream.html')

def stream_main_fun(): 
    
    while True:
        #get camera frame
       
        frame = cv2.imread('./Camera/test_scripts/stream/stream.jpg')
        
        if frame is not None:
            
            frame = cv2.imencode('.JPEG', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[1].tobytes()
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        

@app.route('/video_feed')
def video_feed():
    return Response(stream_main_fun(), mimetype = 'multipart/x-mixed-replace; boundary=frame')

def stream_start():
    # defining server ip address and port
    app.run(debug = True, host = '192.168.1.112', port = '1234', threaded = True)
    
def on_press(key):
    if key == keyboard.KeyCode.from_char('q'): 
        return False 
    return True


stream_start()
