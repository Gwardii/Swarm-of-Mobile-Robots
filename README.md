<img src="Readme_img/logo.png" align="left" style = "width: 10rem"  />

# TGS - The Great Swarm 

Academic project of the swarm of mobile robots - system for controlling and monitoring mobile robots with differential drive. The system consists of a graphical user interface, mobile robot software and software for detecting obstacles. 


## Graphical User Interface
A graphical user interface was created with the use of the Tkinter package. The application allows users to control the mobile robots with the help of control buttons and by assigning the coordinates of the target. In addition, GUI contains a preview of the created map and a preview from the camera mounted above the table where the robots were driving.
![alt text](https://github.com/Gwardii/Swarm-of-Mobile-Robots/blob/master/Readme_img/GUI.png?raw=true)

The robot is finding a path to the target using a diffusion algorithm and raster map. Each cell of the raster map has a unique value of the potential which is dependent on the coordinates of the target and the location of the obstacles. Collaboration between robots was developed using genetic algorithms. 

## Raspberry Pi - vision and detection

Raspberry Pi with a mounted camera detects ArUco markers and procces captured images. Four ArUco markers define corners, robots and obstacles are defined as another sets of markers. Coordiantes of all markers are calculated and send to main server. Live stream from Raspberry is load into local webpage (main application take it for visualization).


<img src=/Readme_img/web_screenshot1.png width=40% ></img> <img src=https://github.com/Gwardii/Swarm-of-Mobile-Robots/blob/master/Readme_img/web_screenshot2.png width=40% align=right></img>

## Mobile robot software
Robots were built around the Arduino Uno boards. Each of them have two wheels equiped with encoders. The communication with the main computer is realized using the XBeeS1 boards. 

<img src=/Readme_img/robot_swarm.jpg width=40%></img>

## Authors

- [Szymon Bielenin](https://github.com/SB-koperkowypiesek)
- [Filip Gwardecki](https://github.com/Gwardii)
- [Bazyli Leczkowski](https://github.com/S4uro0on)
- [Norbert Prokopiuk](https://github.com/norbertprokopiuk)
