<img src="/Readme_img/logo.png" align="left" style = "width: 10rem"  />
<img src="/Readme_img/Obraz4.png" align="right" style = "width: 10rem"  />

## TGS - Robot

This page is all about how to configure robot and communicate it with the server. The robot is shown on the picture on the right. It is based on the Arduino Uno board, has two wheels and communicate with the server via XBee.

## Commands

In order for the robot to move freely around the table to avoid obstacles, it must be able to follow several commands:
1. Stay in the spot for a given amount of time
2. Travel a certain distance forwards and backwards
3. Rotate left or right at a given angle
4. Travel a given distance in a given arc

## Commands execution

<img src="/Readme_img/wykresrobot.png" align="right" style = "width: 40% " />
<br/>

To be able to handle a sworm of robots, the way each one executes a command should be the same. For this to be possible, in addition to movement parameters such as distance or angle, the time for execution of the command is also given. 

When given the command to move, the robot calculates the distance each of its wheels must travel. Then calculates a trapezoidal velocity profile (shown on the figure) with its integral corresponding to the path of the wheel. The angular acceleration of robots is constant and equal 5 * 0.000001 [deg/ms2]. 

<img src="/Readme_img/rownanie.png" align="left" style = "width: 40% " />
<br/>

#### XBEE Data Frame
The XBee board communicates with Arduino using Serial. When the message is received, Serial interrupt is used to stack it an array of robot commands. The frame of the message is shown below and is the same as the one schown in the "computer" part of the project.

Header (0x20 0x40) | Task_ID | Distance (mm) | Task Time (ms)| Radius (mm) | Rotation (deg)| Ender (0x50 0x60)  | Checksum   |
-------------------| --------|---------------|---------------|-------------|---------------|--------------------|------------|
2 bytes            | 1 byte  | 2 bytes       | 3 bytes       | 2 bytes     | 2 bytes       | 2 bytes            | 2 bytes    |

When the robot gets the first message It immediately starts to execute it. When the time allocated to execute the command passes, the robot starts executing the next command.
