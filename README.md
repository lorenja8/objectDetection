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
This file implements all functions into the program environment. Functionalities are assigned to features of the GUI. Camera footage is loaded and projected in the window. Control of movements of individual parts of the robot arm - both automatic and manual - are defined. G-code to guide those movements is generated.

### camera_calibration.py
Its purpose is to find distortion and camera coefficients describing the specific camera used.

### aruco.py
Apart from detecting arbitrary object, the program can identify so called Aruco markers. This functionality is implemented in the file aruco.py. It allows the program to detect a marker, calculate its position with respect to the camera, and in turn calculate the position of the camera with respect to the marker.

### object_detection.py
This file defines methods for detecting objects in a two-channel image, as well as different types of masks to obtain such a two-channel image. The output of those methods is a binary matrix with dimensions corresponding to the image resolution.

### tracker.py
The file defines an Euclidean tracker method (update) which matches object detections on separate frames and matches them to one another. It does so based on the distance of the detections - if an object in one image is detected near an object in a the following image, they are considered the same object. The output of the method is a dictionary of objects and their ids.