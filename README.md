# objectDetection

## Intro
The objectDetection project builds on top of a platform facilitating inverse and forward kinematics and control of an open-chain robot arm with 5 DOF. With a camera mounted to the end effector of the robot, the program is able to identify objects in the robot's proximity, track their position (movements) and calculate their position.

The code operates with the following libraries:
* OpenCV
* NumPy
* PyQt5

Below, individual files and their purpose will be described.

### mainwindow.py and secondary.py
The program's GUI based on the PyQt5 library is defined by those two files. That includes positions, size, colors, etc. of all features found in the window. Each file defines one panel of the window. 

### main.py

### camera_calibration.py

### aruco.py

### object_detection.py

### tracker.py
The file defines an Euclidean tracker method (update) which matches object detections on separate frames and matches them to one another. It does so based on the distance of the detections - if an object in one image is detected near an object in a the following image, they are considered the same object. The output of the method is a dictionary of objects and their ids.