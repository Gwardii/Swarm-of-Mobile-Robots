from states import *
from state_machine import *
import socket

# --------------USER SETTINGS---------------

RPI_IP = "192.168.152.76"
RPI_PORT = 9999

# -----------END OF USER SETTING-----------


class Application(object):
    def __init__(self, cell_size: int = 35, number_of_robots: int = 10, rpi_port: int = 9999, video_feed_ip=0) -> None:

        self.state_machine = StateMachine(self)
        self.states = AllStates
        self.transitions = AllTransition
        if video_feed_ip != 0:
            video_feed_ip = "http://"+video_feed_ip + \
                ":"+str(rpi_port)+"/video_feed"
        # application parameters
        self.cell_size = cell_size
        self.number_of_robots = number_of_robots
        self.gui = GUI(cell_size=cell_size,
                       number_of_robots=number_of_robots, video_feed=video_feed_ip)
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        print(local_ip)
        self.rpi_ip = local_ip
        self.rpi_port = rpi_port

        # initializate all states:
        self._states_initialization(self.states)
        # initializate all transitions:
        self._transition_initialization(self.states, self.transitions)

        # preset flags:
        self.rpi_communicatiom = False
        self.initialization = False
        self.set_target = False
        self.send_data_to_robot = False

    def _states_initialization(self, states: AllStates) -> None:

        # add all states to state_machine (add to states_dictionary):
        self.state_machine.states[states.rpi_communication] = RPI_Communication(
            rpi_ip=self.rpi_ip, port=self.rpi_port)
        self.state_machine.states[states.initialization] = Initialization(
            self.gui)
        self.state_machine.states[states.set_target] = SetTarget(self.gui)
        self.state_machine.states[states.update_camera] = CameraUpdate(
            self.gui)
        self.state_machine.states[states.draw_obstacles] = DrawObstacles(
            self.gui)
        self.state_machine.states[states.draw_path] = DrawPath(self.gui)
        self.state_machine.states[states.robot_control] = RobotControl(
            self.gui)

    def _transition_initialization(self, states: AllStates, transitions: AllTransition) -> None:
        # add all transitions to state_machine (add to transitions_dictionary):
        self.state_machine.transitions[transitions.start_rpi_communication] = Transition(
            states.rpi_communication)
        self.state_machine.transitions[transitions.start_init] = Transition(
            states.initialization)
        self.state_machine.transitions[transitions.set_target] = Transition(
            states.set_target)
        self.state_machine.transitions[transitions.update_camera] = Transition(
            states.update_camera)
        self.state_machine.transitions[transitions.draw_obstacles] = Transition(
            states.draw_obstacles)
        self.state_machine.transitions[transitions.draw_path] = Transition(
            states.draw_path)
        self.state_machine.transitions[transitions.robot_control] = Transition(
            states.robot_control)

    def change_state(self, transition: str) -> None:
        self.state_machine.Transition(transition)
        self.state_machine.Execute()

    def set_state(self, state: str) -> None:
        self.state_machine.Set_state(state)
        self.state_machine.Execute()


def main():

    app = Application(video_feed_ip=RPI_IP,
                      rpi_port=RPI_PORT, number_of_robots=1)
    # start comunnication with raspberry pi:
    app.set_state(app.states.rpi_communication)
    app.rpi_communicatiom = True
    # start initializations:
    gui = app.change_state(app.transitions.start_init)
    app.initialization = True
    # draw obstacles:,
    app.change_state(app.transitions.draw_obstacles)
    # draw first iteration of path
    app.change_state(app.transitions.draw_path)
    tic = time.time()*1000
    while True:
        # ====================
        # some if statement to update widgets
        if app.rpi_communicatiom == True:
            app.gui.rpi_diode.configure(image=app.gui.diode["green"])
        if app.gui.xbee_ready == True:
            app.gui.robot_diode.configure(image=app.gui.diode["green"])
        if app.gui.new_robot_target == True:
            app.change_state(app.transitions.set_target)
            app.change_state(app.transitions.draw_path)
            
        # app.change_state(app.transitions.start_robot_communication)

        # app.send_data_to_robot=True

        if (time.time()*1000-tic > 30):
            app.change_state(app.transitions.robot_control)
            tic = time.time()*1000
        app.change_state(app.transitions.update_camera)

        # update aplication's window:
        app.gui.window.update_idletasks()
        app.gui.window.update()



if __name__ == "__main__":
    main()
