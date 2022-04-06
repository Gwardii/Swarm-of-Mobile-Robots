from concurrent.futures import thread
import string
from turtle import update
from GUI import GUI
from communication.RPI_server import RPI_Communication_Server
import json
from state_machine import StateMachine
import time
import threading

#State base class
class State(object):
    def Execute(self):
        pass

#classes for all gui states
class RPI_Communication(State):
    def __init__(self,rpi_ip:string="localhost",port:int=9999) -> None:
        super().__init__()
        self.rpi_ip = rpi_ip
        self.rpi_port=port
        self.rpi_server=RPI_Communication_Server(self.rpi_ip,self.rpi_port)
    def Execute(self):
        while self.rpi_server.is_rpi_connected==False:
            print("Waiting for established communication", end="\r")
        is_area_received=False
        area_json=None
        obstacles_json=None
        is_obstacle_received=False
        tik=time.time()
        err_num=0
        while not is_area_received:
            if time.time()-tik >=2:
                print("Application did not receive Area JSON")
                err_num+=1
                break
            if self.rpi_server.message_received==True:
                area_json=self.rpi_server.get_buffer()
                is_area_received=True
                break
        tik=time.time()
        while not is_obstacle_received:
            if time.time()-tik >=2:
                print("Application did not receive obstacles JSON")
                err_num+=1
                break
            if self.rpi_server.message_received==True:
                obstacles_json=self.rpi_server.get_buffer()
                is_obstacle_received=True
                break
        if(err_num==0):
            with open(".\Computer\\resources\\obstacles.json","w") as af:
                json.dump(area_json,af)
            with open(".\Computer\\resources\\area.json","w") as af:
                json.dump(obstacles_json,af)

class Initialization(State):
    ## States
    gui_initialization="gui_initialization"
    map_initialization="map_initialization"
    camera_initialization="camera_initialization"
    ##transitions
    start_gui_initialization="start_"+gui_initialization
    start_map_initialization="start_"+gui_initialization
    start_camera_initialization="start_"+camera_initialization
    def __init__(self,gui:GUI) -> None:
        super().__init__()
        self.state_machine=StateMachine(self)
        self.gui = gui
        self._states_initialization()
        self._transition_initalization()

    class GUIInitialization(State):
        def __init__(self,gui:GUI) -> None:
            self.gui=gui

        def Execute(self)->None:
            self.gui.window_configuration()
            print("gui initialization")
            
    
    class MapInitialization(State):
        def __init__(self,gui:GUI) -> None:
            self.gui=gui
        def Execute(self)->None:
            self.gui.draw_figure()
            print("map initialization")
    
    class CameraInitization(State):
        def __init__(self,gui:GUI) -> None:
            self.gui=gui
        def Execute(self)->None:
            self.gui.video_stream()
            print("camera initialization")

    class Transition(object):
        def __init__(self,to_state) -> None:
            self.to_state=to_state
        def Execute(self):
            print("Application is switching to the next state")

    def _states_initialization(self)->None:
        self.state_machine.states[self.gui_initialization]=self.GUIInitialization(gui=self.gui)
        self.state_machine.states[self.map_initialization]=self.MapInitialization(gui=self.gui)
        self.state_machine.states[self.camera_initialization]=self.CameraInitization(gui=self.gui)
    
    def _transition_initalization(self)->None:
        self.state_machine.transitions[self.start_gui_initialization]=self.Transition(self.gui_initialization)
        self.state_machine.transitions[self.start_map_initialization]=self.Transition(self.map_initialization)
        self.state_machine.transitions[self.start_camera_initialization]=self.Transition(self.camera_initialization)
    
    def _change_state(self,transition:str)->None:
        self.state_machine.Transition(transition)
        self.state_machine.Execute()

    def _set_state(self,state:str)->None:
        self.state_machine.Set_state(state)
        self.state_machine.Execute()

    def Execute(self)->GUI:
        self._set_state(self.gui_initialization)
        self._change_state(self.start_map_initialization)
        self._change_state(self.start_camera_initialization)

class CameraUpdate(State):
    def __init__(self,gui:GUI) -> None:
        self.gui = gui
    def Execute(self):
        self.gui.video_stream()

class DrawObstacles(State):
    def __init__(self,gui:GUI) -> None:
        self.gui=gui
    def Execute(self):
        print("Draw obstacle")
        self.gui.draw_obstacles()

class DrawPath(State):
    def __init__(self,gui:GUI) -> None:
        self.gui=gui
    def Execute(self):
        self.gui.draw_path()

class RobotControl(State):
    def __init__(self,gui:GUI) -> None:
        self.gui=gui
    def Execute(self):
        self.gui.robot_control()

class SetTarget(State):
    def __init__(self,gui:GUI) -> None:
        self.gui=gui
    def Execute(self):
        self.gui.background=self.gui.background_without_path
        self.gui.draw_path()

class SendDataToRobot(State):
    def Execute(self):
        pass
        # print("Wysylam dane do robota")

#===========================================
# Add a new state to this struct for easier usage in application's main function
class AllStates():
    rpi_communication="rpi_communication"
    initialization="init"
    set_target="target"
    robot_communication="robot_communication"
    update_camera="update_camera"
    draw_obstacles="draw_obstacles"
    draw_path="draw_path"
    robot_control="robot_control"
#===========================================
# Add a new transition to this struct for easier usage in application's main function
class AllTransition():
    start_rpi_communication="start_rpi_communication"
    start_init="start_init"
    set_target="set_target"
    start_robot_communication="start_robot_communication"
    update_camera="update_camera"
    draw_obstacles="draw_obstacles"
    draw_path="draw_path"
    robot_control="robot_control"

#==========================================
class Transition(object):
    def __init__(self,to_state) -> None:
        self.to_state=to_state
    def Execute(self):
        print("Application is switching to the next state")

