<img src="/Readme_img/logo.png" align="left" style = "width: 10rem"  />
<img src="/Readme_img/Raspberry_Pi_Logo.svg" align="right" style = "width: 9rem"  />

# TGS - Raspbery PI

This page is all about how to configure and set up Raspberry PI, OpenCV. Also there is descriptions about functions and programs which runs on RPI.

<br/>

## Set up Raspberry Pi

In our case RPI is 3B+ but probably most of the advise will be valid on different models.

- install Rasbian (OS for your RPI) follow original page: https://www.raspberrypi.com/software/
- enable: SSH, VNC

### How to get IP of your RPI?
To use your RPI only with your computer you need to get IP of your RPI. You can connect monitor, mouse and keyboard, login and get IP with comand "ifconfig",
also if you enabled VNC IP will be shown if you click on VNC icon in right up corner. But these methods are not really usefull, if you want to connect to new wifi
or you just don't have monitor and mouse and keyboard with you.\
<br/>
So instead, you want to connect with your raspberrypi with ethernet cable and next connect with SSH to your RPI.\
You can use PuTTy:
<br/>
<img src="/Readme_img/putty.png" style = "width: 40% "  align=left />
<br/>
or Windows terminal:
```
> ssh pi@raspberrypi
```
<br/>
or VNC Viewer (preferred method - now, you can use remote desktop!):
<br/>
<img src="/Readme_img/VNC.png" style = "width: 55%" /> 

\* if "raspberrypi" doesn't work you can try "raspberrypi.local"

Next, you can check for IP of your RPI:

```
pi@raspberrypi:~ $ ifconfig
```
Also, at this point you can connect to wifi and read IP. 

Once you get IP of your RPI in network you can disconnect ethernet cable and use one of previous methods to connect with your RPI wireless. 

<img src="/Readme_img/python_opencv.png" style = "width: 40%" align = right /> 

## Install OpenCV

There are a lot of tutorials how to do this and if you manage to do this with them good for you! 
But if you are not familiar with RPI and python and linux and OpenCV (as I was) maybe these tips will be helpfull and save some of your time.
At begining, a tutorial which I suggest to follow: [link](https://pyimagesearch.com/2018/09/26/install-opencv-4-on-your-raspberry-pi/).
With some changes this was the right one. Follow steps and:
- expand filesystem
- unistall unnecessary programs (optional)
- install Cmake and all libraries and packeges (be aware of versions which you install)
- install python3
- download OpenCV 4.X.X - you can download leatest version just change in all command 4.0.0 to your version (we used 4.5.5)
- configure python virtual enviroment (optional) 
- install NumPy
- setup compile with CMake and run cmake to compile OpenCV - check if you don't have any errors! 
- increase swap space: [link](https://pimylifeup.com/raspberry-pi-swap-file/) (use this steps)
- compile OpenCV 4.X.X: make -j4 (if sth crash restart RPI go to this directory and type: make -j1, compilation will be continued) with -j1 it can last even >10 hrs
- after compilation install OpenCV (check if there are no errors!)
- reduce swap space (back to 100 MB): [link](https://pimylifeup.com/raspberry-pi-swap-file/)
- Last step (different as in tutorial)
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
  
  The most important functionality of RPI is detecting Aruco markers and saving their coordinates. But before you want use any camera with opencv you should calibrate  it.
  It is necessary step to reduce fish-eye effect and to achive good results in next steps.  
  
  ### Calibration
  
  First of all you need to take few pictures of calibration grid. You can use special script to do this:
  ```
  $ python ./Camera/capture_frame.py
  ```
  You can use any grid you want ([link](https://markhedleyjones.com/projects/calibration-checkerboard-collection)) but you need to set your parameters in ```       camera_calibration.py```:
  ```
  # Define the dimensions of checkerboard
  # how much corners can be found:
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
  
  Main program for aruco detection has 2 options:
  -  -d (display) if 0 there will be no display window, preffered if you want increase FPS (default is 1)
  -  -js (json sending) if 0 json files will not being send to server (default is 1)
  
  it is important to run this from correct directory:
  ```
  $ python ./Camera/aruco_detection.py -d 1 -js 0
  ```
  Also, if you want to connect with main server (send jsons), it is needed to set correct IP and port of server at beginning of aruco_detection.py:
  ```
  # --------------USER SETTINGS---------------

  server_IP = "192.168.12.120"
  server_PORT = 9999

  # camera setting:
  resolution = [640, 480]
  FPS = 40
  ```
  
  ### stream.py
  
  For visualization and HTML learning simple website was created. Pictures are streamed into this website from where main aplication can feed its display.
  To turn on stream type:
  ```
  $ python ./Camera/stream_tests/flask_version/stream.py
  ```
  
