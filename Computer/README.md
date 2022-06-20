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

GUI was created to provide some feedback, visualization and also interaction with Swarm. The final version of GUI is presented in the photo below. 

<img src="/Readme_img/GUI_2.png" align="center" style = "width: 80%"  />

GUI allows controlling each robot by controlling buttons and also by setting target coordinates. In addition, the app provides a live camera view and information about communication status. The simple terminal enables updating the displayed map by the "update" command. It also can be used to display messages to help debug application. 

### MAP GENERATION

The displayed map is created based on JSON files received from RPI, and information about all possible obstacles read from the database. The application receives three JSON files: area.json, obstacles.json and robot.json. The area.json contains coordinates of all working area corners. Obstacles.json contains coordinates (x, y, rotation) and id of detected obstacles.  Robots.json contains coordinates (x,y, rotation) and id of detected robots. In the next step, the map is rasterized because path generation is based on a diffusion algorithm. The map is divided into cells with a given dimension (dimension can be set in application settings). Later each cell's weight is calculated and saved to a 2D array. These values are later used for path computation and visualisation of cells state (gradient informing about the distance to obstacles) 

### PATH COMPUTING
Path planning algorithm is divided into 3 steps.
The first is some kind of pre-processing. Once a raster map is obtained, its cells are characterised by their distance from the nearest obstacle or workspace boundary. Next, for each of the robots in this map, available cells are determined based on the radius of the robot. If the robot does not have a circular shape it should be written in a circle and then represented by it. The first step should be processed at the begining or whenever the map changes.
The second step may be called a solver. It is simply an implementation of a diffusion alghorithm for path finding. Before moving on, naively check if a path can be traced as a single line between a target and an actual location. If so, return the line as the path and do not include the robot in the diffusion path searching. For each robot an artificial potential field is created. The field has the highest value in the target cell and its values propagates from the cell onto another cells like a wave - assign neighbouring cells lower values and repeat for not assigned cells yet. Next, path is determined as a chain of cells where the first cell is an actual position of a robot and each successive cell is neighbour with the highest value.
Last, but not least is preprocessing. The obtained paths are lists of particular cells, however the most optimal way to store and transmit paths is to describe them as polygonal chain. For the purpose a simple recursive minimalization of a route is implemented.

<img src="/Readme_img/PP_algorithm.png" align="center" style = "width: 80%"  />

### ROBOT COMMUNICATION 
#### XBEE INITIALIZATION
Communication with robots was created with the XBEE module. Before usage, each module has to be initialized. You can do it using XCTU application provided by the producer. To do it you have to plug XBEE module into your computer, launch XCTU and find connected devices by clicking on the "Discover devices" icon in the upper left corner. Next set all data in the displayed window. When your XBEE is discovered you have to change some settings viewed in the table on the right side. Values to change:

- Channel = C
- PAN ID = CAFE
- AP API Enable = API enabled [1]
         
All used XBEE have to be initialized. It has to be done only at your first usage. When all modules are initialized connect XBEE to a robot and in XCTU app click the icon "Find devices in network". When modules were found PC can exchange data with each module. 

#### XBEE Data Frame
To communicate with XBEE digi.xbee Python library was used. This library contains functions to communicate with XBEE in an "easy" way. To communicate with XBEE string or byte array can be used. In this project decided to use a byte array. Each message was prepared on the basis of the frame in the table below:

Header (0x20 0x40) | Task_ID | Distance (mm) | Task Time (ms)| Radius (mm) | Rotation (deg)| Ender (0x50 0x60)  | Checksum   |
-------------------| --------|---------------|---------------|-------------|---------------|--------------------|------------|
2 bytes            | 1 byte  | 2 bytes       | 3 bytes       | 2 bytes     | 2 bytes       | 2 bytes            | 2 bytes    |

It's important to mention that by default, XBee adds 8 bytes of preamble with the destination device address. It can be helpful if you want to use a broadcast mode of communication but in this project unicast mode was used so information about an address was ignored. Without this knowledge communication with XBEE can be tricky.
