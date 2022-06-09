
from RPI_client import RPI_Communication_Client
import json
import time
import threading
from aruco_detection import Aruco_markers, Camera
import cv2
import numpy as np
import math
import argparse
from pynput import keyboard


# --------------USER SETTINGS---------------

server_IP = "192.168.0.31"
server_PORT = 9999

# camera setting:
resolution = [640, 480]
FPS = 40

# table real sizes;
# assumption that table is rectangle and longer side is on x axis (width on photo)
# enter real-world values [mm] for side where 3rd, 2nd markers and 3rd, 0th markers are:
TABLE_SIZE = {'long side': 1800, 'short side': 1400}

# define which markers you use as robots, obstacles, and corners:
robots_markers_ids = [19, 18, 12, 13]
obstacles_markers_ids = [7, 9, 10, 14, 15, 17]
area_markers_ids = [0, 1, 2, 3]

# define real size of a side of markers (in [mm]):
marker_real_size = 335

# -----------END OF USER SETTING-----------


class StateMachine(object):
    def __init__(self) -> None:
        self.states = {}
        self.transitions = {}
        self.current_state = State()
        self.current_transition: Transition = None
        self.previous_state = self.current_state
        self.is_alive = True

    def Set_state(self, state_name) -> None:
        self.current_state = self.states[state_name]

    def Transition(self, transition_name) -> None:
        self.current_transition = self.transitions[transition_name]

    def Execute(self):
        if(self.current_transition):
            # self.current_transition.Execute()
            self.Set_state(self.current_transition.to_state)
            self.current_transition = None
        self.current_state.Execute()
        self.Transition(self.current_state.transition)


# State base class
class State(object):
    def __init__(self) -> None:
        self.name = None
        self.transition = None

    def Execute(self):
        pass


# ===========================================
# Add a new state to this struct:
class AllStates():
    # names of all states:
    initialization = "initialization"
    aruco_searching = "aruco_searching"
    jsons_saving = "jsons_saving"
    jsons_sending = "jsons_sending"
    adding_images = "adding_images"
    saving_displaying_stream = "saving_displaying_stream"
    checking_stop_conditions = "checking_stop_conditions"
    end_of_program = "end_of_program"


# ===========================================
# Add a new transition to this struct:
class AllTransition():
    # names of all transitions:
    start_initialization = "start_initialization"
    find_aruco = "find_aruco"
    save_jsons = "save_jsons"
    send_jsons = "send_jsons"
    add_images = "add_images"
    save_display_stream = "save_display_stream"
    check_if_stop = "check_if_stop"
    end_program = "end_program"


class Transition(object):
    def __init__(self, to_state) -> None:
        self.to_state = to_state

    def Execute(self):
        print("Application is switching to the next state")


# ===========================================
# Classes for states:

class Initialization(State):

    def __init__(self, aruco: Aruco_markers, camera: Camera, key_checker: keyboard.Listener) -> None:
        super().__init__()
        self.name = "initialization"
        self.aruco = aruco
        self.camera = camera
        self.key_checker = key_checker

    def Execute(self):

        self.aruco.aruco_Cam_param = self.camera.camera_calibration_params
        # start separate thread:
        self.camera.start()

        self.aruco.load_Aug_Images("./Camera/images")

        # stop script conditions:
        print("\npress:\n- q to quit")
        # start listening for keyboard
        self.key_checker.start()

        self.transition = "find_aruco"


class Aruco_searching(State):

    def __init__(self, aruco: Aruco_markers, camera: Camera) -> None:
        super().__init__()
        self.name = "aruco_searching"
        self.aruco = aruco
        self.camera = camera

    def Execute(self):
        if(self.aruco.find_Aruco_Markers(self.camera.take_frame(), draw=False)):
            self.transition = "save_jsons"
            return
        self.transition = "save_display_stream"


class Jsons_saving(State):

    def __init__(self, aruco: Aruco_markers, args) -> None:
        super().__init__()
        self.name = "json_saving"
        self.aruco = aruco
        self.args = args

    def Execute(self):
        self.aruco.save_Aruco_json()
        if self.args["json_sending"] > 0:
            self.transition = "send_jsons"
            return
        self.transition = "add_images"


class Jsons_sending(State):

    def __init__(self, aruco: Aruco_markers, client: RPI_Communication_Client) -> None:
        super().__init__()
        self.name = "jsons_sending"
        self.aruco = aruco
        self.client = client

    def Execute(self):

        self.client.send_json(
            json.load(open("./Computer/resources/area.json", 'r')))
        self.client.send_json(
            json.load(open("./Computer/resources/obstacles.json", 'r')))
        self.client.send_json(
            json.load(open("./Computer/resources/robots.json", 'r')))

        self.transition = "add_images"


class Images_adding(State):

    def __init__(self, aruco: Aruco_markers) -> None:
        super().__init__()
        self.name = "adding_images"
        self.aruco = aruco

    def Execute(self):
        self.aruco.augment_Aruco(self.aruco.main_IMG, drawId=False)
        self.transition = "save_display_stream"


class Stream_saving_displaying(State):

    def __init__(self, aruco: Aruco_markers, camera: Camera, args) -> None:
        super().__init__()
        self.name = "saving_displaying_stream"
        self.aruco = aruco
        self.camera = camera
        self.args = args

    def Execute(self):

        # save stream video:
        cv2.imwrite(
            './Camera/stream_tests/flask_version/stream/stream.jpg', self.aruco.main_IMG)
        if self.args["display"] > 0:
            cv2.imshow("ARUCO DETECTION", self.aruco.main_IMG)
            cv2.waitKey(1)

        self.transition = "check_if_stop"


class Stop_checking(State):
    def __init__(self, camera: Camera, k: keyboard.Listener) -> None:
        super().__init__()
        self.name = "checking_stop_conditions"
        self.camera = camera
        self.key_checker = k

    def Execute(self):
        if self.key_checker.is_alive():
            self.transition = "find_aruco"
            return

        # if quiting key was pressed:
        print('\nquitting...\n')
        self.camera.stop()
        self.transition = "end_program"


class End_program(State):
    def __init__(self) -> None:
        super().__init__()
        self.name = "end_of_program"
        self.transition = "end_program"


# construct the argument parse and parse the arguments
# (these will be argument that you can use while calling aruco_detection.py):
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--display", type=int, default=1,
                help="Whether or not frames should be displayed")
ap.add_argument("-js", "--json_sending", type=int, default=1,
                help="Whether or not you want to stream json files")
args = vars(ap.parse_args())


# Additional functions:
def on_press(key):
    if key == keyboard.KeyCode.from_char('q'):
        return False
    return True


def main():

    aruco_markers = Aruco_markers()

    # start video capturing
    camera = Camera(0, resolution[0], resolution[1], FPS)

    # start client for json sending:
    # enter IP of server:
    if args["json_sending"] > 0:
        client = RPI_Communication_Client(host=server_IP, port=server_PORT)

    # for keyboard hanling:
    key_checker = keyboard.Listener(on_press=on_press)

    # create and preapare state machine:

    state_machine = StateMachine()
    states = AllStates()
    transitions = AllTransition()

    # add all states to dictionary:
    state_machine.states[states.initialization] = Initialization(
        aruco_markers, camera, key_checker)
    state_machine.states[states.aruco_searching] = Aruco_searching(
        aruco_markers, camera)
    state_machine.states[states.jsons_saving] = Jsons_saving(
        aruco_markers, args)
    state_machine.states[states.jsons_sending] = Jsons_sending(
        aruco_markers, client)
    state_machine.states[states.adding_images] = Images_adding(
        aruco_markers)
    state_machine.states[states.saving_displaying_stream] = Stream_saving_displaying(
        aruco_markers, camera, args)
    state_machine.states[states.checking_stop_conditions] = Stop_checking(
        camera, key_checker)
    state_machine.states[states.end_of_program] = End_program()

    # add all transitions to dictionary:
    state_machine.transitions[transitions.start_initialization] = Transition(
        states.initialization)
    state_machine.transitions[transitions.find_aruco] = Transition(
        states.aruco_searching)
    state_machine.transitions[transitions.save_jsons] = Transition(
        states.jsons_saving)
    state_machine.transitions[transitions.send_jsons] = Transition(
        states.jsons_sending)
    state_machine.transitions[transitions.add_images] = Transition(
        states.adding_images)
    state_machine.transitions[transitions.save_display_stream] = Transition(
        states.saving_displaying_stream)
    state_machine.transitions[transitions.check_if_stop] = Transition(
        states.checking_stop_conditions)
    state_machine.transitions[transitions.end_program] = Transition(
        states.end_of_program)

    # start main:
    state_machine.current_transition = state_machine.transitions[transitions.start_initialization]

    while(state_machine.current_state.name != "end_of_program"):
        state_machine.Execute()


if __name__ == '__main__':
    main()
