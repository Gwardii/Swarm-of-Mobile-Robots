# Swarm of mobile robots

Academic project of the swarm of mobile robots - system for controlling and monitoring mobile robots with differential drive. The system consists of a graphical user interface, mobile robot software and software for detecting obstacles. 


## Graphical User Interface
A graphical user interface was created with the use of the Tkinter package. The application allows users to control the mobile robots with the help of control buttons and by assigning the coordinates of the target. In addition, GUI contains a preview of the created map and a preview from the camera mounted above the table where the robots were driving.
![alt text](https://github.com/Gwardii/Swarm-of-Mobile-Robots/blob/master/Readme_img/GUI.png?raw=true)

The robot is finding a path to the target using a diffusion algorithm and raster map. Each cell of the raster map has a unique value of the potential which is dependent on the coordinates of the target and the location of the obstacles. Collaboration between robots was developed using genetic algorithms. 

## Obstacles detection
The system detects obstacles using Raspberry Pi with a mounted camera. The camera script captures images and detects ArUco markers. Four ArUco markers were using to calculate the coordinate system. Others were attached to the obstacles and with help calculated coordinate system position and rotation of obstacles were defined.  
## Mobile robot software
## Authors

- [Szymon Bielenin](https://github.com/SB-koperkowypiesek)
- [Filip Gwardecki](https://github.com/Gwardii)
- [Bazyli Leczkowski](https://github.com/S4uro0on)
- [Norbert Prokopiuk](https://github.com/norbertprokopiuk)
