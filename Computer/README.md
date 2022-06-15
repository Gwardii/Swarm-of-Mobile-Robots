<img src="/Readme_img/logo.png" align="left" style = "width: 10rem"  />

### TGS - MAIN COMPUTER

This page is about the main computer which is a brain of a whole swarm. It computes paths, decides of movements of the robots, and communicates with them. Also, it runs the main GUI and receives data from RPI. Below, there are descriptions of functions and programs which runs on the main computer.

<br/>

### application.py

Main sript which starts all the others. After starting the application initialization starts: the application will try to communicate with RPI and receive JSONs from it. After 10 [s] without receiving data it will be terminated and GUI starts to generate. 
Only setting which you need to do is to provide IP, PORT of RPI, number of used robots and XBEE mode at the beginning of the program:
```
# --------------USER SETTINGS---------------

RPI_IP = "192.168.12.120" #set 0 to use your computer camera
RPI_PORT = 9999
NUMBER_OF_ROBOTS = 1
XBEE = True

# -----------END OF USER SETTING-----------
```
#### State machine

In order to provide more orderly and readable code state machine was implemented. All states and possible transitions are shown on the diagram below:

<img src="/Readme_img/main_state_machine_2.png" align="center" style = "width: 80%"  />

All states are implemented as separate classes inherited from a state base class. In this way, all classes have a similar structure and it's easy to figure out what is the task of each state. In addition structure like this allows for easy and quick adding new states. It is important to mention that a state also can be a separated state machine containing some smaller states. In this project, the state Initialization was implemented as another state machine so if you want to implement something like this refer to the class Initialization in states.py. 
The same concept of state machine was used in the Camera state machine so, in order to don't repeat the same information, only this one has a detailed description of an action. 

### Adding new states
In order to add a new state, you have to create a new class inherited from the class State. In the next step in the Execute function put the code of actions performed by this new state. 
Later add your state to the AllStates and the AllTransitions classes, which contain names of all possible states and transitions. At the end add your state to state and transition initialization. In order to that go to application.py and in the _states_initialization and _transition_initialization initialize a new class like classes already added.

### GUI

GUI was created to provide some feedback, visualization and also interaction with Swarm. 

<img src="/Readme_img/GUI_2.png" align="center" style = "width: 80%"  />



### PATH COMPUTING
Main idea behind creating paths for robots is......




