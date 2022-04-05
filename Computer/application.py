from re import S
from states import *
from state_machine import *
import time 
class Application(object):
    def __init__(self,cell_size:int=50,number_of_robots:int=10,rpi_ip:string="localhost",rpi_port:int=9999) -> None:
        self.state_machine=StateMachine(self)
        self.states=AllStates
        self.transitions=AllTransition
        #application parameters
        self.cell_size=cell_size
        self.number_of_robots=number_of_robots
        self.gui=GUI(cell_size=cell_size,number_of_robots=number_of_robots)
        self.rpi_ip=rpi_ip
        self.rpi_port=rpi_port

        self._states_initialization(self.states)
        self._transition_initalization(self.states,self.transitions)
        self.rpi_communicatiom = False
        self.initialization=False
        self.set_target=False
        self.send_data_to_robot=False

    def _states_initialization(self, states:AllStates)->None:
        self.state_machine.states[states.rpi_communication]=RPI_Communication(rpi_ip=self.rpi_ip,port=self.rpi_port)
        self.state_machine.states[states.initialization]=Initialization(self.gui)
        self.state_machine.states[states.set_target]=SetTarget(self.gui)
        self.state_machine.states[states.robot_communication]=SendDataToRobot()
        self.state_machine.states[states.update_camera]=CameraUpdate(self.gui)
        self.state_machine.states[states.draw_obstacles]=DrawObstacles(self.gui)
        self.state_machine.states[states.draw_path]=DrawPath(self.gui)
        self.state_machine.states[states.robot_control]=RobotControl(self.gui)

    def _transition_initalization(self,states:AllStates,transitions:AllTransition)->None:
        self.state_machine.transitions[transitions.start_rpi_communication]=Transition(states.rpi_communication)
        self.state_machine.transitions[transitions.start_init]=Transition(states.initialization)
        self.state_machine.transitions[transitions.set_target]=Transition(states.set_target)
        self.state_machine.transitions[transitions.start_robot_communication]=Transition(states.robot_communication)
        self.state_machine.transitions[transitions.update_camera]=Transition(states.update_camera)
        self.state_machine.transitions[transitions.draw_obstacles]=Transition(states.draw_obstacles)
        self.state_machine.transitions[transitions.draw_path]=Transition(states.draw_path)
        self.state_machine.transitions[transitions.robot_control]=Transition(states.robot_control)

    def change_state(self,transition:str)->None:
        self.state_machine.Transition(transition)
        self.state_machine.Execute()
    def set_state(self,state:str)->None:
        self.state_machine.Set_state(state)
        self.state_machine.Execute()

if __name__ == "__main__":
    app=Application()
    
    app.set_state(app.states.rpi_communication)
    app.rpi_communicatiom=True
    gui=app.change_state(app.transitions.start_init)
    app.initialization=True
    app.change_state(app.transitions.draw_obstacles)
    app.change_state(app.transitions.draw_path)
    while True:
        #====================
        #some if statement to update widgets
        if app.rpi_communicatiom==True:
            app.gui.rpi_diode.configure(image=app.gui.diode["green"])
        if app.gui.new_robot_target==True:
            app.change_state(app.transitions.set_target)
        # app.change_state(app.transitions.start_robot_communication)
        # app.send_data_to_robot=True
        app.change_state(app.transitions.robot_control)
        app.change_state(app.transitions.update_camera)
        app.gui.window.update_idletasks()
        app.gui.window.update()

