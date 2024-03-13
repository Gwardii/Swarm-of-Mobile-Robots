<img src="/Readme_img/logo.png" align="left" style = "width: 10rem"  />
<img src="/Readme_img/Raspberry_Pi_Logo.svg" align="right" style = "width: 9rem"  />

# TGS - Raspberry PI

This page is all about how to configure and set up Raspberry PI, and OpenCV. Also, there are descriptions of functions and programs which run on RPI.

<br/>

## Set up Raspberry Pi

In our case RPI is 3B+ but probably most of the advice will be valid on different models.

- install Rasbian (OS for your RPI) by following the original page: https://www.raspberrypi.com/software/
- enable: SSH, VNC

### How do you get the IP of your RPI?
To use your RPI only with your computer you need to get the IP of your RPI. You can connect the monitor, mouse and keyboard, log in and get IP with the command "ifconfig",
also if you enabled VNC IP will be shown if you click on the VNC icon in the right-up corner. But these methods are not useful if you want to connect to new wifi
or you just don't have a monitor and mouse and keyboard with you.\
<br/>
So instead, you want to connect with your raspberrypi with an ethernet cable and next connect with SSH to your RPI.\
You can use PuTTy:
<br/>
<img src="/Readme_img/putty.png" style = "width: 40% "  align=left />
<br/>
or Windows terminal:
```
> ssh pi@raspberrypi
```
<br/>
or VNC Viewer (preferred method - now, you can use a remote desktop!):
<br/>
<img src="/Readme_img/VNC.png" style = "width: 55%" /> 

\* if "raspberrypi" doesn't work you can try "raspberrypi.local"

Next, you can check for the IP of your RPI:

```
pi@raspberrypi:~ $ ifconfig
```
Also, at this point, you can connect to wifi and read IP. 

Once you get the IP of your RPI in the network you can disconnect the ethernet cable and use one of the previous methods to connect with your RPI wireless. 

<img src="/Readme_img/python_opencv.png" style = "width: 40%" align = right /> 

## Installing OpenCV

There are a lot of tutorials on how to do this and if you manage to do this with them good for you! 
But if you are not familiar with RPI, python, Linux and OpenCV (as I was) maybe these tips will be helpful and save some of your time.
At beginning, a tutorial which I suggest to follow: [link](https://pyimagesearch.com/2018/09/26/install-opencv-4-on-your-raspberry-pi/).
With some changes, this was the right one. Follow the steps:
- expand filesystem
- uninstall unnecessary programs (optional)
- install Cmake and all libraries and packages (be aware of versions which you install)
- install python3
- download OpenCV 4.X.X - you can download the latest version just change all commands 4.0.0 to your version (we used 4.5.5)
- Configure Python virtual environment (optional) 
- install NumPy
- setup compile with CMake and run cmake to compile OpenCV - check if you don't have any errors! 
- increase swap space: [link](https://pimylifeup.com/raspberry-pi-swap-file/) (use this steps)
- compile OpenCV 4.X.X: make -j4 (if sth crash restart RPI go to this directory and type: make -j1, compilation will be continued) with -j1 it can last even >10 hrs
- after compilation install OpenCV (check if there are no errors!)
- reduce swap space (back to 100 MB): [link](https://pimylifeup.com/raspberry-pi-swap-file/)
- Last step (different as in the tutorial)
  - find and go into ../python3.[X](https://your_python_version)/site-packages
  - find path for ../python-3.[X](https://your_python_version)/cv2.cpython-3[X](https://your_python_version)m-arm-linux-gnueabihf.so (in our case: "/usr/local/lib/python3.9/site-packages/cv2/python-3.9/cv2.cpython-39-arm-linux-gnueabihf.so"
  - create symbolic link ( ```ln -s ../path_to_XXX.so cv2.so ```)
 - test if OpenCV is working:
 ```
 $ python
 >>>import cv2
 >>>cv2.__version__
  ```
  If you receive your OpenCV version you are ready to use it!
  
  ## Aruco detection
  
  The most important functionality of RPI is detecting Aruco markers and saving their coordinates. But     before you want to use any camera with OpenCV you should calibrate  it.
  It is a necessary step to reduce fish-eye effect and to achieve good results in the next steps.  
  Also, it is worth knowing that Aruco markers should have some light frame around a black one or be mounted on a light background (necessary for detection of the black square).  
  ### Calibration
  
  First of all, you need to take a few pictures of the calibration grid. You can use a special script to do this:
  ```
  $ python ./Camera/capture_frame.py
  ```
  You can use any grid you want ([link](https://markhedleyjones.com/projects/calibration-checkerboard-collection)) but you need to set your parameters in ```       camera_calibration.py```:
  ```
  # Define the dimensions of the checkerboard
  # How many corners can be found:
  # horizontal and vertical
  # also size of the side of a square in [mm])
  CHECKERBOARD = (8, 5, 251/9)
  ```
  Next, you can run:```$ python ./Camera/camera_calibration.py``` and
  calibration file will be saved as "./Camera/camera_calibration.npy".
  Example with correctly detected grid:
  <br/>
  <img src="/Readme_img/calibration.png" style = "width: 40%" align=right/> 
  
  
  ### aruco_detection.py
  
  The main program for aruco detection has 2 options:
  -  -d (display) if 0 there will be no display window, preferred if you want to increase FPS (default is 1)
  -  -js (json sending) if 0 json files will not be sent to the server (default is 1)
  
  it is important to run this from the correct directory:
  ```
  $ python ./Camera/aruco_detection.py -d 1 -js 0
  ```
  Also, if you want to connect with the main server (send jsons), it is needed to set the correct IP and port of   the server at the beginning of aruco_detection.py:
  ```
  # --------------USER SETTINGS---------------

  server_IP = "192.168.12.120"
  server_PORT = 9999

  # camera setting:
  resolution = [640, 480]
  FPS = 40
  ```
  
  ### state_machine_RPI.py
  
aruco_detection.py based on "if" statements which was good enough but with the development of this project new functionality will be added. Also remembering all flags is not so easy after some time. So, we decided to create a Finite State Machine (FSM). This concept is really useful to organize all functionality and allows to add in easy way new ones. In a nutshell, FSM is based on states (which all are defined at the beginning) and transitions which allow for changes between states. Each state has arbitrarily defined all possible transitions. FSM usually doesn't provide less code but its purpose is to give more orderly and clear code. For more info about FSM see [link](https://brilliant.org/wiki/finite-state-machines/), [link](https://www.youtube.com/watch?v=2OiWs-h_M3A). 
 <img src="/Readme_img/Aruco state machine.png" style = "width: 55% "  align=right />
 <br/>
 
All states and transitions are shown on this diagram. Each of the states consist of the main function and at the end possible transitions. The main while loop runs only one command and on each iteration, you can check on which state you are in. It is probably not the best solution but for sure it is useful to know the FSM concept and own implementation is the best way to learn this. 

In use there is no difference with aruco_detection.py still all options work:
```
 $ python .\Camera\state_machine_RPI.py -d 0 -js 1   
```
  ### stream.py
  
  For visualization and HTML learning, a simple website was created. Pictures are streamed into this website   from where the main application can feed its display.
  To turn on stream type:
  ```
  $ python ./Camera/stream_tests/flask_version/stream.py
  ```
  
