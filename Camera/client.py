import socketio

sio = socketio.Client()

def send_sensor_reading():
    while True:
        sio.emit('my_message',{'temp':75})
        sio.sleep(5)

@sio.event
def connect():
    print('connection established')
    sio.start_background_task(send_sensor_reading)

@sio.event
def disconnect():
    print('disconnected from server')

sio.connect('http://192.168.137.212:5000')
