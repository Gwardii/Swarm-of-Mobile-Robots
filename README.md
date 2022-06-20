<img src="Readme_img/logo.png" align="left" style = "width: 10rem"  />

# TGS - The Great Swarm 

Academic project of the swarm of mobile robots - system for controlling and monitoring mobile robots with differential drive. The system consists of a graphical user interface, mobile robot software and software for detecting obstacles. 


## Graphical User Interface
A graphical user interface was created with the use of the Tkinter package. The application allows users to control the mobile robots with the help of control buttons and by assigning the coordinates of the target. In addition, GUI contains a preview of the created map and a preview from the camera mounted above the table where the robots were driving.
![alt text](https://github.com/Gwardii/Swarm-of-Mobile-Robots/blob/master/Readme_img/GUI_2.png?raw=true)

## Raspberry Pi - vision and detection

Raspberry Pi with a mounted camera detects ArUco markers and procces captured images. Four ArUco markers define corners, robots and obstacles are defined as another sets of markers. Coordiantes of all markers are calculated and send to main server. Live stream from Raspberry is load into local webpage (main application take it for visualization).


<img src=/Readme_img/web_screenshot1.png width=40% ></img> <img src=https://github.com/Gwardii/Swarm-of-Mobile-Robots/blob/master/Readme_img/web_screenshot2.png width=40% align=right></img>

## Mobile robot software
Robots were built around the Arduino Uno boards. Each of them has two wheels equipped with encoders. The communication with the central computer is realized using the XBeeS1 boards.

<img src=/Readme_img/robot_swarm.jpg width=40%></img>

A robot can get the command to wait, move forward, rotate, and move around the given arc. Each command is built so the robot will drive a given distance or angle in a given amount of time.
The main goal while programming robots was to make their path as deterministic as possible. To achieve this, each command consists of the path parametrs such as its length or radius and the time in with the path should be complited. Based on that robot calculates the path for each wheel and follows it using PID.

## Path planning
Implemented path planning algorithm finds the shortest feasible polygonal chain between target and actual position of a robot. The path planner requires a rasterized map of working area, thus it was also covered in the project. At the moment, the rasterization is provided for any polygon and circle, which are should be provided in the specified json format.

## Authors

- [Szymon Bielenin](https://github.com/SB-koperkowypiesek)
- [Filip Gwardecki](https://github.com/Gwardii)
- [Bazyli Leczkowski](https://github.com/S4uro0on)
- [Norbert Prokopiuk](https://github.com/norbertprokopiuk)
