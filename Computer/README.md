<img src="/Readme_img/logo.png" align="left" style = "width: 10rem"  />

### TGS - MAIN COMPUTER

This page is about the main computer which is a brain of a whole swarm. It computes paths, decides of movements of the robots, and communicates with them. Also, it runs the main GUI and receives data from RPI. Below, there are descriptions of functions and programs which runs on the main computer.

<br/>

### application.py

Main sript which starts all the others. After starting the application initialization starts: the application will try to communicate with RPI and receive JSONs from it. After 10 [s] without receiving data it will be terminated and GUI starts to generate. 
Only setting which you need to do is to provide IP and PORT of RPI at the beginning of the program:
```
# --------------USER SETTINGS---------------

RPI_IP = "192.168.12.120"
RPI_PORT = 9999

# -----------END OF USER SETTING-----------
```
#### State machine

In order to provide more orderly and readable code state machine was implemented. All states and possible transitions are shown on the diagram below:

<img src="/Readme_img/main_state_machine.svg" align="center" style = "width: 80%"  />



### GUI

GUI was created to provide some feedback, visualization and also interaction with Swarm.  

<img src="/Readme_img/GUI.png" align="center" style = "width: 80%"  />


### PATH COMPUTING
Main idea behind creating paths for robots is......




