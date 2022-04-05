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
        self.state_machine.states[states.initialization]=Initialization()
        self.state_machine.states[states.set_target]=SetTarget()
        self.state_machine.states[states.robot_communication]=SendDataToRobot()

    def _transition_initalization(self,states:AllStates,transitions:AllTransition)->None:
        self.state_machine.transitions[transitions.start_rpi_communication]=Transition(states.rpi_communication)
        self.state_machine.transitions[transitions.start_init]=Transition(states.initialization)
        self.state_machine.transitions[transitions.set_target]=Transition(states.set_target)
        self.state_machine.transitions[transitions.start_robot_communication]=Transition(states.robot_communication)

    def change_state(self,transition:str)->None:
        self.state_machine.Transition(transition)
        self.state_machine.Execute()
    def set_state(self,state:str)->None:
        self.state_machine.Set_state(state)
        self.state_machine.Execute()

if __name__ == "__main__":
    app=Application()
    
    app.set_state(app.states.rpi_communication)
    time.sleep(1)
    app.rpi_communicatiom=True
    app.change_state(app.transitions.start_init)
    app.initialization=True
    time.sleep(1)
    app.change_state(app.transitions.set_target)
    app.set_target=True
    time.sleep(1)
    app.change_state(app.transitions.start_robot_communication)
    app.send_data_to_robot=True

