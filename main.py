import math
import sys
import time

import cv2 as cv
import numpy as np
import serial
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

import serial_port as sp
from aruco import calibrator
from mainwindow import Ui_MainWindow
from object_detection import detector
from secondary import Ui_Form
from tracker import tracker

ser = serial.Serial()

theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = 0

class Secondary(Ui_Form):            # defines the Settings window and its functions
    def __init__(self, parent=None):
        Ui_Form.__init__(self)
        self.setupUi(parent)

        self.settings_slider1.valueChanged.connect(self.update)
        self.settings_slider2.valueChanged.connect(self.update)
        self.settings_slider3.valueChanged.connect(self.update)
        self.settings_slider4.valueChanged.connect(self.update)

        self.settings_default_pushbutton.clicked.connect(self.default)

    def update(self):
        p1 = self.settings_slider1.value()
        p2 = self.settings_slider2.value()
        p3 = self.settings_slider3.value()
        p4 = self.settings_slider4.value()
        parameters = (p1, p2, p3, p4)
        return parameters

    def default(self):
        self.settings_slider1.setValue(160)
        self.settings_slider2.setValue(255)
        self.settings_slider3.setValue(240)
        self.settings_slider4.setValue(255)

    def preview_1(self, frame):
        frame = cv.resize(frame, (480, 270))
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        frame = QtGui.QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QtGui.QImage.Format_RGB888)
        self.settings_footage_label1.setPixmap(QtGui.QPixmap.fromImage(frame))

    def preview_2(self, frame):
        frame = cv.resize(frame, (480, 270))
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        frame = QtGui.QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QtGui.QImage.Format_RGB888)
        self.settings_footage_label2.setPixmap(QtGui.QPixmap.fromImage(frame))


class Application(Ui_MainWindow):
    def show_new_window(self):
        self.window_class.show()

    def __init__(self, parent=None):
        Ui_MainWindow.__init__(self)
        self.setupUi(parent)

        # dialog window
        self.cv_settings_pushbutton.clicked.connect(self.show_new_window)
        self.window_class = QtWidgets.QWidget()
        self.window = Secondary(self.window_class)

        ######### ZACH ##########
        # Serial port
        self.serialportcombobox()
        self.refresh_pushbutton.pressed.connect(self.serialportcombobox)
        self.connect_pushbutton.pressed.connect(self.serialportconnection)

        # Wrist 2 Movement
        self.wrist2_pushbutton1.pressed.connect(self.wrist2dec5)
        self.wrist2_pushbutton2.pressed.connect(self.wrist2dec1)
        self.wrist2_pushbutton3.pressed.connect(self.wrist2dec0_1)
        self.wrist2_pushbutton4.pressed.connect(self.wrist2inc0_1)
        self.wrist2_pushbutton5.pressed.connect(self.wrist2inc1)
        self.wrist2_pushbutton6.pressed.connect(self.wrist2inc5)
        self.wrist2_slider.valueChanged.connect(self.wrist2slider)
        self.wrist2_dial.valueChanged.connect(self.wrist2dial)
        self.wrist2_spinbox_2.valueChanged.connect(self.wrist2spinbox2)
        self.wrist2_spinbox_2.valueChanged.connect(self.wrist2ok)
        # self.wrist2_pushbuttonok.pressed.connect(self.wrist2ok)

        # Wrist 1 Movement
        self.wrist1_pushbutton1.pressed.connect(self.wrist1dec5)
        self.wrist1_pushbutton2.pressed.connect(self.wrist1dec1)
        self.wrist1_pushbutton3.pressed.connect(self.wrist1dec0_1)
        self.wrist1_pushbutton4.pressed.connect(self.wrist1inc0_1)
        self.wrist1_pushbutton5.pressed.connect(self.wrist1inc1)
        self.wrist1_pushbutton6.pressed.connect(self.wrist1inc5)
        self.wrist1_slider.valueChanged.connect(self.wrist1slider)
        self.wrist1_dial.valueChanged.connect(self.wrist1dial)
        self.wrist1_spinbox_2.valueChanged.connect(self.wrist1spinbox2)
        self.wrist1_spinbox_2.valueChanged.connect(self.wrist1ok)
        # self.wrist1_pushbuttonok.pressed.connect(self.wrist1ok)

        # Elbow Movement
        self.elbow_pushbutton1.pressed.connect(self.elbowdec5)
        self.elbow_pushbutton2.pressed.connect(self.elbowdec1)
        self.elbow_pushbutton3.pressed.connect(self.elbowdec0_1)
        self.elbow_pushbutton4.pressed.connect(self.elbowinc0_1)
        self.elbow_pushbutton5.pressed.connect(self.elbowinc1)
        self.elbow_pushbutton6.pressed.connect(self.elbowinc5)
        self.elbow_slider.valueChanged.connect(self.elbowslider)
        self.elbow_dial.valueChanged.connect(self.elbowdial)
        self.elbow_spinbox_2.valueChanged.connect(self.elbowspinbox2)
        self.elbow_spinbox_2.valueChanged.connect(self.elbowok)
        # self.elbow_pushbuttonok.pressed.connect(self.elbowok)

        # Shoulder Movement
        self.shoulder_pushbutton1.pressed.connect(self.shoulderdec5)
        self.shoulder_pushbutton2.pressed.connect(self.shoulderdec1)
        self.shoulder_pushbutton3.pressed.connect(self.shoulderdec0_1)
        self.shoulder_pushbutton4.pressed.connect(self.shoulderinc0_1)
        self.shoulder_pushbutton5.pressed.connect(self.shoulderinc1)
        self.shoulder_pushbutton6.pressed.connect(self.shoulderinc5)
        self.shoulder_slider.valueChanged.connect(self.shoulderslider)
        self.shoulder_dial.valueChanged.connect(self.shoulderdial)
        self.shoulder_spinbox_2.valueChanged.connect(self.shoulderspinbox2)
        self.shoulder_spinbox_2.valueChanged.connect(self.shoulderok)
        # self.shoulder_pushbuttonok.pressed.connect(self.shoulderok)

        # Base Movement
        self.base_pushbutton1.pressed.connect(self.basedec5)
        self.base_pushbutton2.pressed.connect(self.basedec1)
        self.base_pushbutton3.pressed.connect(self.basedec0_1)
        self.base_pushbutton4.pressed.connect(self.baseinc0_1)
        self.base_pushbutton5.pressed.connect(self.baseinc1)
        self.base_pushbutton6.pressed.connect(self.baseinc5)
        self.base_slider.valueChanged.connect(self.baseslider)
        self.base_dial.valueChanged.connect(self.basedial)
        self.base_spinbox_2.valueChanged.connect(self.basespinbox2)
        self.base_spinbox_2.valueChanged.connect(self.baseok)
        # self.base_pushbuttonok.pressed.connect(self.baseok)

        # Gripper Control
        self.gripper_pushbutton1.pressed.connect(self.gripperdec5)
        self.gripper_pushbutton2.pressed.connect(self.gripperdec1)
        self.gripper_pushbutton3.pressed.connect(self.gripperinc1)
        self.gripper_pushbutton4.pressed.connect(self.gripperinc5)
        self.gripper_slider.valueChanged.connect(self.gripperslider)
        self.gripper_spinbox.valueChanged.connect(self.gripperspinbox)
        self.gripper_spinbox.valueChanged.connect(self.gripperok)
        # self.gripper_pushbuttonok.pressed.connect(self.gripperok)

        # Inverse Kinematics
        self.inversekinematics_ok.pressed.connect(self.inversekinematics)
        self.x_pushbutton.pressed.connect(self.xpush)
        self.x_pushbutton2.pressed.connect(self.xpush2)
        self.y_pushbutton.pressed.connect(self.ypush)
        self.y_pushbutton2.pressed.connect(self.ypush2)
        self.z_pushbutton.pressed.connect(self.zpush)
        self.z_pushbutton2.pressed.connect(self.zpush2)

        ######### LORENC ##########
        # Localization and movements
        self.T = list                    # translation matrix from base to end effector, also coordinates of the end effector
        self.R = list                    # rotation matrix from base to end effector
        self.view_angle = float          # viewing angle of the camera in relation to the ground
        self.object = []                 # coordinates of the object
        self.target = [0]*3              # coordinates of the target placement location
        self.extruder = 0                # wrist1 and wrist2 defined as extruders 1 and 0
        self.direction_vector = (0, 0)   # the direction from the object to the frame center
        self.limit_value = 8             # the distance in px at which the robot and the object are considered aligned
        self.moveswitch = False          # switches automatic movement ON/OFF
        self.going_right = True          # determines
        self.home_pushbutton.clicked.connect(self.move_to_home)
        self.target_pushbutton.clicked.connect(self.move_to_target)
        self.lookout_pushbutton.clicked.connect(self.move_to_lookout)

        # Aruco
        self.calibrate = calibrator()    # instance of the calibrate class from aruco.py
        self.calibswitch = False         # calibration switch (True = calibration running)
        self.cv_calibration_pushbutton.clicked.connect(self.switch_aruco)
        self.cv_arucosearch_radiobutton2.setChecked(True)

        # Detection
        self.buttons_ids = {}            # dictionary of displayed buttons by their ids
        self.detectswitch = False        # detection switch (True = detection running)
        self.picked_id = None            # id of the object to be picked by the robot
        self.cv_operation_pushbutton.clicked.connect(self.instantiate_detection)
        self.cv_operation_radiobutton2.setChecked(True)

        # Camera
        self.cap = None
        self.camswitch = False           # camera switch (True = video footage running)
        self.cv_refresh_pushbutton.clicked.connect(self.test_camera)
        self.cv_serialport_combobox.currentIndexChanged.connect(self.test_camera)
        self.cv_camswitch.clicked.connect(self.footage)

    ######### ZACH ##########
    # Serial port
    def serialportcombobox(self):
        self.serialport_combobox.clear()
        self.serialport_combobox.addItems(sp.serial_ports())
        self.serial_label.setText("Disconnected")
        self.serial_label.setStyleSheet("QLabel { color : red }")
        self.serial_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))

    def serialportconnection(self):
        serialport = self.serialport_combobox.currentText()
        baudrate = self.baudrate_combobox.currentText()
        if serialport != "":
            ser.port = serialport
            ser.baudrate = baudrate
            ser.timeout = 1
            try:
                ser.close()
                ser.open()
                self.serial_label.setText("Connected")
                self.serial_label.setStyleSheet("QLabel { color : green }")
                bytes = ser.inWaiting()
                ser.read(bytes)
                print(bytes)
                print(ser.read(bytes))
            except:
                self.serial_label.setText("No connection")
                self.serial_label.setStyleSheet("QLabel { color : red }")
        else:
            self.serial_label.setText("Error - Serial value")
            self.serial_label.setStyleSheet("QLabel { color : red }")

    # Wrist 2 Movement
    def wrist2dec5(self):
        self.wrist2_spinbox_2.setValue(self.wrist2_spinbox_2.value() - 5)

    def wrist2dec1(self):
        self.wrist2_spinbox_2.setValue(self.wrist2_spinbox_2.value() - 1)

    def wrist2dec0_1(self):
        self.wrist2_spinbox_2.setValue(self.wrist2_spinbox_2.value() - 0.1)

    def wrist2inc0_1(self):
        self.wrist2_spinbox_2.setValue(self.wrist2_spinbox_2.value() + 0.1)

    def wrist2inc1(self):
        self.wrist2_spinbox_2.setValue(self.wrist2_spinbox_2.value() + 1)

    def wrist2inc5(self):
        self.wrist2_spinbox_2.setValue(self.wrist2_spinbox_2.value() + 5)

    def wrist2slider(self):
        refactored_for_spinbox = self.wrist2_slider.value() / 10
        self.wrist2_spinbox_2.setValue(refactored_for_spinbox)

    def wrist2dial(self):
        self.wrist2_slider.setValue(self.wrist2_dial.value())

    def wrist2spinbox2(self):
        refactored_for_slider = self.wrist2_spinbox_2.value() * 10
        self.wrist2_slider.setValue(int(refactored_for_slider))
        self.wrist2_dial.setValue(int(refactored_for_slider))

    def wrist2ok(self):
        if ser.isOpen():
            if self.extruder == 1:
                self.extruder = 0
                ser.write("T0\n".encode())
                time.sleep(0)
            if self.wrist2_radiobutton1.isChecked():
                move = "G00 " + "E" + str(int(self.wrist2_spinbox_2.value())) + " F1000" + "\n"
            else:
                move = "G01 " + "E" + str(self.wrist2_spinbox_2.value()) + " F" + str(
                    round(self.wrist2_spinbox.value(), 1)) + "\n"
            print(move)
            bytes = ser.inWaiting()
            ser.read(bytes)
            time.sleep(0)
            ser.write(move.encode())
            time.sleep(0)
            print(ser.readline().decode("ascii"))
            self.wrist2_labelactvalue.setText(str(self.wrist2_spinbox_2.value()) + "°")
            self.wrist2_labelactvalue.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
            self.forwardkinematics()
        else:
            self.serial_label.setText("No connection")
            self.serial_label.setStyleSheet("QLabel { color : red }")

    # Wrist 1 Movement
    def wrist1dec5(self):
        self.wrist1_spinbox_2.setValue(self.wrist1_spinbox_2.value() - 5)

    def wrist1dec1(self):
        self.wrist1_spinbox_2.setValue(self.wrist1_spinbox_2.value() - 1)

    def wrist1dec0_1(self):
        self.wrist1_spinbox_2.setValue(self.wrist1_spinbox_2.value() - 0.1)

    def wrist1inc0_1(self):
        self.wrist1_spinbox_2.setValue(self.wrist1_spinbox_2.value() + 0.1)

    def wrist1inc1(self):
        self.wrist1_spinbox_2.setValue(self.wrist1_spinbox_2.value() + 1)

    def wrist1inc5(self):
        self.wrist1_spinbox_2.setValue(self.wrist1_spinbox_2.value() + 5)

    def wrist1slider(self):
        refactored_for_spinbox = self.wrist1_slider.value() / 10
        self.wrist1_spinbox_2.setValue(refactored_for_spinbox)

    def wrist1dial(self):
        self.wrist1_slider.setValue(self.wrist1_dial.value())

    def wrist1spinbox2(self):
        refactored_for_slider = self.wrist1_spinbox_2.value() * 10
        self.wrist1_slider.setValue(int(refactored_for_slider))
        self.wrist1_dial.setValue(int(refactored_for_slider))

    def wrist1ok(self):
        if ser.isOpen():
            if self.extruder == 0:
                self.extruder = 1
                ser.write("T1\n".encode())
                time.sleep(0)
            if self.wrist1_radiobutton1.isChecked():
                move = "G00 " + "E" + str(self.wrist1_spinbox_2.value()) + " F1000" + "\n"
            else:
                move = "G01 " + "E" + str(self.wrist1_spinbox_2.value()) + " F" + str(
                    round(self.wrist1_spinbox.value(), 1)) + "\n"
            ser.write(move.encode())
            time.sleep(0)
            print(ser.readline().decode("ascii"))
            self.wrist1_labelactvalue.setText(str(self.wrist1_spinbox_2.value()) + "°")
            self.wrist1_labelactvalue.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
            self.forwardkinematics()
        else:
            self.serial_label.setText("No connection")
            self.serial_label.setStyleSheet("QLabel { color : red }")

    # Elbow Movement
    def elbowdec5(self):
        self.elbow_spinbox_2.setValue(self.elbow_spinbox_2.value() - 5)

    def elbowdec1(self):
        self.elbow_spinbox_2.setValue(self.elbow_spinbox_2.value() - 1)

    def elbowdec0_1(self):
        self.elbow_spinbox_2.setValue(self.elbow_spinbox_2.value() - 0.1)

    def elbowinc0_1(self):
        self.elbow_spinbox_2.setValue(self.elbow_spinbox_2.value() + 0.1)

    def elbowinc1(self):
        self.elbow_spinbox_2.setValue(self.elbow_spinbox_2.value() + 1)

    def elbowinc5(self):
        self.elbow_spinbox_2.setValue(self.elbow_spinbox_2.value() + 5)

    def elbowslider(self):
        refactored_for_spinbox = self.elbow_slider.value() / 10
        self.elbow_spinbox_2.setValue(refactored_for_spinbox)

    def elbowdial(self):
        self.elbow_slider.setValue(self.elbow_dial.value())

    def elbowspinbox2(self):
        refactored_for_slider = self.elbow_spinbox_2.value() * 10
        self.elbow_slider.setValue(int(refactored_for_slider))
        self.elbow_dial.setValue(int(refactored_for_slider))

    def elbowok(self):
        if ser.isOpen():
            if self.elbow_radiobutton1.isChecked():
                move = "G00 " + "Y" + str(self.elbow_spinbox_2.value()) + " F1000" + "\n"
            else:
                move = "G01 " + "Y" + str(self.elbow_spinbox_2.value()) + " F" + str(
                    round(self.elbow_spinbox.value(), 1)) + "\n"
            ser.write(move.encode())
            bytes = ser.inWaiting()
            ser.read(bytes)
            time.sleep(0)
            print(ser.readline().decode("ascii"))
            self.elbow_labelactvalue.setText(str(self.elbow_spinbox_2.value()) + "°")
            self.elbow_labelactvalue.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
            self.forwardkinematics()
        else:
            self.serial_label.setText("No connection")
            self.serial_label.setStyleSheet("QLabel { color : red }")

    # Shoulder Movement
    def shoulderdec5(self):
        self.shoulder_spinbox_2.setValue(self.shoulder_spinbox_2.value() - 5)

    def shoulderdec1(self):
        self.shoulder_spinbox_2.setValue(self.shoulder_spinbox_2.value() - 1)

    def shoulderdec0_1(self):
        self.shoulder_spinbox_2.setValue(self.shoulder_spinbox_2.value() - 0.1)

    def shoulderinc0_1(self):
        self.shoulder_spinbox_2.setValue(self.shoulder_spinbox_2.value() + 0.1)

    def shoulderinc1(self):
        self.shoulder_spinbox_2.setValue(self.shoulder_spinbox_2.value() + 1)

    def shoulderinc5(self):
        self.shoulder_spinbox_2.setValue(self.shoulder_spinbox_2.value() + 5)

    def shoulderslider(self):
        refactored_for_spinbox = self.shoulder_slider.value() / 10
        self.shoulder_spinbox_2.setValue(refactored_for_spinbox)

    def shoulderdial(self):
        self.shoulder_slider.setValue(self.shoulder_dial.value())

    def shoulderspinbox2(self):
        refactored_for_slider = self.shoulder_spinbox_2.value() * 10
        self.shoulder_slider.setValue(int(refactored_for_slider))
        self.shoulder_dial.setValue(int(refactored_for_slider))

    def shoulderok(self):
        if ser.isOpen():
            if self.shoulder_radiobutton1.isChecked():
                move = "G00 " + "Z" + str(self.shoulder_spinbox_2.value()) + " F1000" + "\n"
            else:
                move = "G01 " + "Z" + str(self.shoulder_spinbox_2.value()) + " F" + str(
                    round(self.shoulder_spinbox.value(), 1)) + "\n"
            ser.write(move.encode())
            time.sleep(0)
            print(ser.readline().decode("ascii"))
            self.shoulder_labelactvalue.setText(str(self.shoulder_spinbox_2.value()) + "°")
            self.shoulder_labelactvalue.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
            self.forwardkinematics()
        else:
            self.serial_label.setText("No connection")
            self.serial_label.setStyleSheet("QLabel { color : red }")

    # Base Movement
    def basedec5(self):
        self.base_spinbox_2.setValue(self.base_spinbox_2.value() - 5)

    def basedec1(self):
        self.base_spinbox_2.setValue(self.base_spinbox_2.value() - 1)

    def basedec0_1(self):
        self.base_spinbox_2.setValue(self.base_spinbox_2.value() - 0.1)

    def baseinc0_1(self):
        self.base_spinbox_2.setValue(self.base_spinbox_2.value() + 0.1)

    def baseinc1(self):
        self.base_spinbox_2.setValue(self.base_spinbox_2.value() + 1)

    def baseinc5(self):
        self.base_spinbox_2.setValue(self.base_spinbox_2.value() + 5)

    def baseslider(self):
        refactored_for_spinbox = self.base_slider.value() / 10
        self.base_spinbox_2.setValue(refactored_for_spinbox)

    def basedial(self):
        self.base_slider.setValue(self.base_dial.value())

    def basespinbox2(self):
        refactored_for_slider = self.base_spinbox_2.value() * 10
        self.base_slider.setValue(int(refactored_for_slider))
        self.base_dial.setValue(int(refactored_for_slider))

    def baseok(self):
        if ser.isOpen():
            if self.base_radiobutton1.isChecked():
                move = "G00 " + "X" + str(int(self.base_spinbox_2.value())) + " F1000" + "\n"
            else:
                move = "G01 " + "X" + str(self.base_spinbox_2.value()) + " F" + str(
                    round(self.base_spinbox.value(), 1)) + "\n"
            print(move)
            bytes = ser.inWaiting()
            ser.read(bytes)
            time.sleep(0)
            ser.write(move.encode())
            time.sleep(0)
            print(ser.readline().decode("ascii"))
            self.base_labelactvalue.setText(str(self.base_spinbox_2.value()) + "°")
            self.base_labelactvalue.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
            self.forwardkinematics()
        else:
            self.serial_label.setText("No connection")
            self.serial_label.setStyleSheet("QLabel { color : red }")

    # Gripper Control
    def gripperdec5(self):
        self.gripper_spinbox.setValue(self.gripper_spinbox.value() - 5)

    def gripperdec1(self):
        self.gripper_spinbox.setValue(self.gripper_spinbox.value() - 1)

    def gripperinc1(self):
        self.gripper_spinbox.setValue(self.gripper_spinbox.value() + 1)

    def gripperinc5(self):
        self.gripper_spinbox.setValue(self.gripper_spinbox.value() + 5)

    def gripperslider(self):
        self.gripper_spinbox.setValue(self.gripper_slider.value())

    def gripperspinbox(self):
        self.gripper_slider.setValue(int(self.gripper_spinbox.value()))

    def gripperok(self):
        if ser.isOpen():
            move = "M280 P0" + "S" + str((self.gripper_spinbox.value()*120/100)) + "\n"
            ser.write(move.encode())
            bytes = ser.inWaiting()
            ser.read(bytes)
            time.sleep(0)
            print(ser.readline().decode("ascii"))
            self.gripper_labelactvalue.setText(str(self.base_spinbox_2.value()) + "°")
            self.gripper_labelactvalue.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
        else:
            self.serial_label.setText("No connection")
            self.serial_label.setStyleSheet("QLabel { color : red }")

    # Forward Kinematics
    def forwardkinematics(self):

        theta1rad = -(self.base_spinbox_2.value() / 180) * np.pi + np.pi
        theta2rad = (self.shoulder_spinbox_2.value() / 180) * np.pi + np.pi / 2
        theta3rad = (self.elbow_spinbox_2.value() / 180) * np.pi + np.pi / 2
        theta4rad = (self.wrist1_spinbox_2.value() / 180) * np.pi
        theta5rad = (self.wrist2_spinbox_2.value() / 180) * np.pi

        dhmatrix = [[theta1rad, 231.5, 0, np.pi / 2],
                    [theta2rad, 0, 221.5, 0],
                    [theta3rad, 0, 0, np.pi / 2],
                    [theta4rad, 224.5, 0, -np.pi / 2],
                    [theta5rad, 0, 0, np.pi / 2],
                    [0, 98, 0, 0]]

        t01 = [[np.cos(dhmatrix[0][0]), -np.sin(dhmatrix[0][0]) * np.cos(dhmatrix[0][3]),
                np.sin(dhmatrix[0][0]) * np.sin(dhmatrix[0][3]), dhmatrix[0][2] * np.cos(dhmatrix[0][0])],
               [np.sin(dhmatrix[0][0]), np.cos(dhmatrix[0][0]) * np.cos(dhmatrix[0][3]),
                -np.cos(dhmatrix[0][0]) * np.sin(dhmatrix[0][3]), dhmatrix[0][2] * np.sin(dhmatrix[0][0])],
               [0, np.sin(dhmatrix[0][3]), np.cos(dhmatrix[0][3]), dhmatrix[0][1]],
               [0, 0, 0, 1]]

        t12 = [[np.cos(dhmatrix[1][0]), -np.sin(dhmatrix[1][0]) * np.cos(dhmatrix[1][3]),
                np.sin(dhmatrix[1][0]) * np.sin(dhmatrix[1][3]), dhmatrix[1][2] * np.cos(dhmatrix[1][0])],
               [np.sin(dhmatrix[1][0]), np.cos(dhmatrix[1][0]) * np.cos(dhmatrix[1][3]),
                -np.cos(dhmatrix[1][0]) * np.sin(dhmatrix[1][3]), dhmatrix[1][2] * np.sin(dhmatrix[1][0])],
               [0, np.sin(dhmatrix[1][3]), np.cos(dhmatrix[1][3]), dhmatrix[1][1]],
               [0, 0, 0, 1]]

        t23 = [[np.cos(dhmatrix[2][0]), -np.sin(dhmatrix[2][0]) * np.cos(dhmatrix[2][3]),
                np.sin(dhmatrix[2][0]) * np.sin(dhmatrix[2][3]), dhmatrix[2][2] * np.cos(dhmatrix[2][0])],
               [np.sin(dhmatrix[2][0]), np.cos(dhmatrix[2][0]) * np.cos(dhmatrix[2][3]),
                -np.cos(dhmatrix[2][0]) * np.sin(dhmatrix[2][3]), dhmatrix[2][2] * np.sin(dhmatrix[2][0])],
               [0, np.sin(dhmatrix[2][3]), np.cos(dhmatrix[2][3]), dhmatrix[2][1]],
               [0, 0, 0, 1]]

        t34 = [[np.cos(dhmatrix[3][0]), -np.sin(dhmatrix[3][0]) * np.cos(dhmatrix[3][3]),
                np.sin(dhmatrix[3][0]) * np.sin(dhmatrix[3][3]), dhmatrix[3][2] * np.cos(dhmatrix[3][0])],
               [np.sin(dhmatrix[3][0]), np.cos(dhmatrix[3][0]) * np.cos(dhmatrix[3][3]),
                -np.cos(dhmatrix[3][0]) * np.sin(dhmatrix[3][3]), dhmatrix[3][2] * np.sin(dhmatrix[3][0])],
               [0, np.sin(dhmatrix[3][3]), np.cos(dhmatrix[3][3]), dhmatrix[3][1]],
               [0, 0, 0, 1]]

        t45 = [[np.cos(dhmatrix[4][0]), -np.sin(dhmatrix[4][0]) * np.cos(dhmatrix[4][3]),
                np.sin(dhmatrix[4][0]) * np.sin(dhmatrix[4][3]), dhmatrix[4][2] * np.cos(dhmatrix[4][0])],
               [np.sin(dhmatrix[4][0]), np.cos(dhmatrix[4][0]) * np.cos(dhmatrix[4][3]),
                -np.cos(dhmatrix[4][0]) * np.sin(dhmatrix[4][3]), dhmatrix[4][2] * np.sin(dhmatrix[4][0])],
               [0, np.sin(dhmatrix[4][3]), np.cos(dhmatrix[4][3]), dhmatrix[4][1]],
               [0, 0, 0, 1]]

        t56 = [[np.cos(dhmatrix[5][0]), -np.sin(dhmatrix[5][0]) * np.cos(dhmatrix[5][3]),
                np.sin(dhmatrix[5][0]) * np.sin(dhmatrix[5][3]), dhmatrix[5][2] * np.cos(dhmatrix[5][0])],
               [np.sin(dhmatrix[5][0]), np.cos(dhmatrix[5][0]) * np.cos(dhmatrix[5][3]),
                -np.cos(dhmatrix[5][0]) * np.sin(dhmatrix[5][3]), dhmatrix[5][2] * np.sin(dhmatrix[5][0])],
               [0, np.sin(dhmatrix[5][3]), np.cos(dhmatrix[5][3]), dhmatrix[5][1]],
               [0, 0, 0, 1]]

        t02 = np.dot(t01, t12)
        t03 = np.dot(t02, t23)
        t04 = np.dot(t03, t34)
        t05 = np.dot(t04, t45)
        t06 = np.dot(t05, t56)

        [[rxx, rxy, rxz, tx], [ryx, ryy, ryz, ty], [rzx, rzy, rzz, tz], [_, _, _, _]] = t06
        self.T = [tx, ty, tz]
        self.R = np.array([[rxx, rxy, rxz], [ryx, ryy, ryz], [rzx, rzy, rzz]])

        self.camera_angle(self.R)
        self.set_values(tx, ty, tz)

    def camera_angle(self, R):           # calculates the deviation of the end effector from the z0 axis
        k = np.array([0, 0, 1])          # unit vector in z
        k_rot = R.dot(k)

        dot_product = np.dot(k, k_rot)
        angle_r = math.pi - np.arccos(dot_product)
        angle_d = math.degrees(angle_r)
        self.view_angle = angle_d

    def set_values(self, tx, ty, tz):       # displays calculated values in the GUI

        self.x_pos_label.setText(str(round(tx, 1)))
        self.x_pos_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
        self.y_pos_label.setText(str(round(ty, 1)))
        self.y_pos_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
        self.z_pos_label.setText(str(round(tz, 1)))
        self.z_pos_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))

        # addition for aruco
        self.cv_x_pos_label.setText(str(round(tx, 1)))
        self.cv_x_pos_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
        self.cv_y_pos_label.setText(str(round(ty, 1)))
        self.cv_y_pos_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
        self.cv_z_pos_label.setText(str(round(tz, 1)))
        self.cv_z_pos_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))

    # Inverse Kinematics
    def inversekinematics(self, x=None, y=None, z=None):

        if x is None and y is None and z is None:
            x = self.x_spinbox.value()
            y = self.y_spinbox.value()
            z = self.z_spinbox.value()

        l1 = 231.5
        l2 = 221.5
        l3 = 224.5
        l4 = 98

        theta1 = -np.arctan2(y, x)
        r = np.sqrt(x ** 2 + y ** 2)
        d = np.sqrt((z-l1) ** 2 + r ** 2)
        theta3 = np.arccos((d ** 2 - (l3+l4) ** 2 - l2 ** 2) / (2 * (l3 + l4) * l2))
        theta2 = np.arctan2(r, z-l1) - np.arctan2(l2 + (l3+l4) * np.cos(-theta3), (l3+l4) * np.sin(-theta3))
        print(theta1 * 180 / np.pi, (theta2 + np.pi / 2) * 180 / np.pi, theta3 * 180 / np.pi)
        theta5 = 0
        theta4 = 0

        if theta1 * 180 / np.pi >= -135 and theta1 * 180 / np.pi <= 135:
            if (theta2 + np.pi / 2) * 180 / np.pi >= -90 and (theta2 + np.pi / 2) * 180 / np.pi <= 90:
                if theta3 * 180 / np.pi >= -120 and theta3 * 180 / np.pi <= 120:
                    self.elbow_spinbox_2.setValue(round(theta3 * 180 / np.pi, 1))
                    self.base_spinbox_2.setValue(round(theta1 * 180 / np.pi, 1))
                    self.shoulder_spinbox_2.setValue(round((theta2 + np.pi / 2) * 180 / np.pi, 1))
                    self.wrist2_spinbox_2.setValue(round(theta5 * 180 / np.pi, 1))
                    self.wrist1_spinbox_2.setValue(round(theta4 * 180 / np.pi, 1))
                else:
                    self.z_label.setText("Theta 3 not found")
                    self.z_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
                    self.z_label.setStyleSheet("QLabel { color : red }")
            else:
                self.y_label.setText("Theta 2 not found")
                self.y_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
                self.y_label.setStyleSheet("QLabel { color : red }")
        else:
            self.x_label.setText("Theta 1 not found")
            self.x_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 7, weight=QtGui.QFont.Bold))
            self.x_label.setStyleSheet("QLabel { color : red }")

    def xpush(self):
        value = self.x_vel_spinbox.value() + self.x_spinbox.value()
        self.x_spinbox.setValue(value)

    def xpush2(self):
        value = - self.x_vel_spinbox.value() + self.x_spinbox.value()
        self.x_spinbox.setValue(value)

    def ypush(self):
        value = self.y_vel_spinbox.value() + self.y_spinbox.value()
        self.y_spinbox.setValue(value)

    def ypush2(self):
        value = - self.y_vel_spinbox.value() + self.y_spinbox.value()
        self.y_spinbox.setValue(value)

    def zpush(self):
        value = self.z_vel_spinbox.value() + self.z_spinbox.value()
        self.z_spinbox.setValue(value)

    def zpush2(self):
        value = - self.z_vel_spinbox.value() + self.z_spinbox.value()
        self.z_spinbox.setValue(value)

    ######### LORENC ##########
    # Localization and movements
    def move_to_home(self):           # moves the robot to the upright position and turn off calibration and detection
        self.calibswitch = False
        self.detectswitch = False
        self.setG("00")
        self.base_spinbox_2.setValue(0)
        self.shoulder_spinbox_2.setValue(0)
        self.elbow_spinbox_2.setValue(0)
        self.wrist1_spinbox_2.setValue(0)
        self.wrist2_spinbox_2.setValue(0)

    def move_to_target(self):         # moves the robot to the location a set position
        x, y, z = self.target
        self.inversekinematics(x, y, z)

    def move_to_lookout(self):        # moves the effector to an observing position
        self.shoulder_spinbox_2.setValue(-30)
        self.elbow_spinbox_2.setValue(80)
        self.wrist2_spinbox_2.setValue(80)

    def autosearch(self):             # moves the robot systematically to search the workspace
        if ser.isOpen():
            current = self.base_spinbox_2.value()
            max = 10   # self.base_spinbox_2.maximum()
            min = -10  # self.base_spinbox_2.minimum()
            if self.going_right == False:           # moves between horizontal thresholds
                self.move_horizontally(step=-0.1, speed=270)
            else:
                self.move_horizontally(step=0.1, speed=270)
            if current == max and self.going_right is True:    # when a horizontal threshold is reached, the robot moves
                self.move_vertically(step=-5, speed=270)       # vertically (up) and the horizontal direction changes
                time.sleep(2.5)
                self.going_right = not self.going_right
            elif current == min and self.going_right is False:
                self.move_vertically(step=-5, speed=270)
                time.sleep(2.5)
                self.going_right = not self.going_right

    def setG(self, type="00"):       # sets the type of movement in G-code
        if type == "01":
            self.wrist2_radiobutton2.setChecked(True)
            self.wrist1_radiobutton2.setChecked(True)
            self.elbow_radiobutton2.setChecked(True)
            self.shoulder_radiobutton2.setChecked(True)
            self.base_radiobutton2.setChecked(True)
        else:
            self.wrist2_radiobutton1.setChecked(True)
            self.wrist1_radiobutton1.setChecked(True)
            self.elbow_radiobutton1.setChecked(True)
            self.shoulder_radiobutton1.setChecked(True)
            self.base_radiobutton1.setChecked(True)

    def speed(self, part, speed):       # picks a specified wrist and changes the movement speed
        spinbox = part + "_spinbox"
        result = getattr(self, spinbox)
        result.setValue(speed)

    def step(self, part,  step):        # picks a specified wrist and changes the step size
        spinbox = part + "_spinbox_2"
        result = getattr(self, spinbox)
        old = result.value()
        new = old + step
        result.setValue(new)

    def move_horizontally(self, step, speed):     # moves the base horizontally
        self.setG(type="01")
        part = "base"
        self.speed(part=part, speed=speed)
        self.step(part=part, step=step)

    def move_vertically(self, step, speed):      # chooses either the elbow or wrist2 and moves it vertically
        self.setG(type="01")
        if step > 0 and self.wrist2_spinbox_2.value() == self.wrist2_spinbox_2.maximum():
            part = "elbow"
        elif step < 0 and self.wrist2_spinbox_2.value() == self.wrist2_spinbox_2.minimum():
            part = "elbow"
        else:
            part = "wrist2"
        self.speed(part=part, speed=speed)
        self.step(part=part, step=step)

    def align_frame_with_object(self, object_center):        # finds which way the robot needs to move
        frame = self.frame
        frame_size = np.array([len(frame[0]), len(frame)])
        frame_center = np.divide(frame_size, 2)
        fc_x = frame_center[0]
        fc_y = frame_center[1]
        cv.circle(frame, (int(fc_x), int(fc_y)), 2, (0, 0, 255), 4)

        o_x = object_center[0]          # object/marker center (depending on the application)
        o_y = object_center[1]
        cv.circle(frame, (int(o_x), int(o_y)), 2, (0, 255, 0), 4)

        self.direction_vector = np.subtract(object_center, frame_center)     # vector from the frame center to the object center

    def move_how(self, direction):       # determines the speed, step size and direction of robot movements
        dv_x, dv_y = self.direction_vector
        quick = 50
        if direction == "horizontal":
            if dv_x > quick or dv_x < -quick:
                speed = 360
                step = 0.5
            else:
                speed = 72
                step = 0.1
        else:
            if dv_y > quick or dv_y < -quick:
                speed = 360
                step = 0.5
            else:
                speed = 72
                step = 0.1
        return step, speed

    def move_accordingly(self):         # aligns the camera center with the object center
        if self.moveswitch is True:
            lim = self.limit_value
            dv_x, dv_y = self.direction_vector

            if dv_x > lim or dv_x < -lim:         # aligns the camera horizontally
                step, speed = self.move_how("horizontal")
                step = -step if dv_x > 0 else step
                self.move_horizontally(step, speed)
            elif dv_y > lim or dv_y < -lim:         # aligns the camera vertically
                step, speed = self.move_how("vertical")
                step = -step if dv_y < 0 else step
                self.move_vertically(step, speed)

    def frame_center_to_world_coordinates(self, calibration=True):      # calculates realworld coordinates of the frame center
        self.forwardkinematics()
        tx, ty, tz = self.T
        distance_from_robot = abs(tz * math.tan(math.radians(self.view_angle)))
        base_angle = self.base_spinbox_2.value()

        print("transformacni matice k efektoru:")
        print("x=" + str(tx))
        print("y=" + str(ty))
        print("z=" + str(tz))

        print("vzdalenost od kamery: " + str(distance_from_robot))
        print("odchylka kamery "+str(self.view_angle))
        print(math.cos(math.radians(base_angle)))

        x = round(tx + distance_from_robot * math.cos(math.radians(base_angle)))
        y = round(ty - distance_from_robot * math.sin(math.radians(base_angle)))
        z = 0
        if calibration == True:    # used for calibration or detection, results are stored in lists
            self.target = [x, y, z]
            print(self.target)
            self.cv_x_target_spinbox.setValue(x)
            self.cv_y_target_spinbox.setValue(y)
            self.cv_z_target_spinbox.setValue(z)
        elif calibration == False:
            self.object = [x, y, z]
            self.cv_x_obj_label.setText(str(x))
            self.cv_x_obj_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
            self.cv_x_obj_label.setStyleSheet("QLabel { color : black }")
            self.cv_y_obj_label.setText(str(y))
            self.cv_y_obj_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
            self.cv_y_obj_label.setStyleSheet("QLabel { color : black }")
            self.cv_z_obj_label.setText(str(z))
            self.cv_z_obj_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
            self.cv_z_obj_label.setStyleSheet("QLabel { color : black }")

    # Aruco calibration
    def switch_aruco(self):       # calibration is switched ON/OFF and detection is switched OFF
        self.calibswitch = not self.calibswitch
        self.detectswitch = False
        self.cv_arucosearch_radiobutton1.setChecked(False)

    def aruco_calibration(self):          # explained in aruco.py
        if self.calibswitch is True:
            cv.putText(self.frame, str("Calibrating"), (25, 25), cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 0))
            self.calibrate.detect_markers(self.frame)
            success, reference_marker = self.calibrate.reference_marker(self.cv_arucoid_combobox.currentIndex())


            if success is False:
                self.moveswitch = False
            elif success is True:        # when marker is found, the robot begins to align itself
                self.cv_arucosearch_radiobutton2.setChecked(True)
                self.cv_operation_radiobutton2.setChecked(True)

                self.calibrate.find_pose_single(self.frame, reference_marker)

                corners = len(reference_marker[0])
                sum = np.sum(reference_marker[0][:], axis=0)
                marker_pos = np.divide(sum, corners)
                m_x = marker_pos[0]
                m_y = marker_pos[1]
                cv.circle(self.frame, (int(m_x), int(m_y)), 2, (255, 0, 0), 4)

                self.align_frame_with_object(marker_pos)
                self.moveswitch = True

                lim = self.limit_value
                dv_x, dv_y = self.direction_vector
                if -lim < dv_x < lim and -lim < dv_y < lim:       # when the frame center is close enough to the marker, the calibration stops
                    self.frame_center_to_world_coordinates(calibration=True)
                    self.moveswitch = False
                    self.calibswitch = False
                    self.cv_calibstatus_label.setText(QtCore.QCoreApplication.translate("MainWindow","<html><head/><body><p><span style=\" font-size:10pt; font-weight:600; color: green;\">CALIBRATED</span></p></body></html>"))

    # Object detection
    def instantiate_detection(self):                    # upon pressing a button, calibration is switched OFF and
        self.detectswitch = not self.detectswitch       # detection is switched ON/OFF; a new instance of detector and
        self.calibswitch = False                        # tracker is created, overwriting the previous one
        self.picked_id = None
        self.detect = detector()
        self.track = tracker()

    def object_detection(self):         # explained in object_detection.py
        if self.detectswitch is True:
            copy = self.frame.copy()
            cv.putText(self.frame, str("Detecting"), (25, 25), cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 0))
            self.parameters = self.window.update()

            self.detect.parameters = self.parameters
            if self.window.settings_method_combobox.currentText() == "Threshold":    # chooses the detection method
                gray, mask = self.detect.method_1(copy)
                self.window.preview_1(gray)
                self.window.preview_2(mask)
            else:
                mask = self.detect.method_4(copy)
                self.window.preview_1(mask)
                self.window.preview_2(mask)
            self.detect.detect_contours(mask)

            self.track.update(self.detect.ellipses)
            self.track.draw_ellipse(self.frame)

            self.existing(self.track.ellipses_ids)
            if self.picked_id is not None:        # if an id is picked, the robot is aligned with the appropriate object
                self.cv_arucosearch_radiobutton2.setChecked(True)
                self.cv_operation_radiobutton2.setChecked(True)
                if self.picked_id not in self.track.ellipses_ids:
                    self.moveswitch = False
                    self.picked_id = None
                else:
                    ellipse_center = self.track.ellipses_ids[self.picked_id][0]

                    self.align_frame_with_object(ellipse_center)
                    self.moveswitch = True

                    lim = self.limit_value
                    if -lim < self.direction_vector[0] < lim and -lim < self.direction_vector[1] < lim:    # robot stops when the object is close enough
                        self.frame_center_to_world_coordinates(calibration=False)
                        self.moveswitch = False
                        self.picked_id = None

    def existing(self, ellipses_ids):          # checks if a button already exists, otherwise a new button is created
        for key_ellipse in ellipses_ids:
            if key_ellipse not in self.buttons_ids:
                self.create_button(key_ellipse)
        for key_button in self.buttons_ids.copy():
            if key_button not in ellipses_ids:
                self.buttons_ids[key_button].setParent(None)
                self.buttons_ids.pop(key_button)

    def create_button(self, id):              # creates buttons with ids of detected objects
        Button = QtWidgets.QPushButton(str(id))
        Button.clicked.connect(lambda: self.pick_id(Button))

        self.buttons_ids[id] = Button
        self.gridLayout_18.addWidget(Button)

    def pick_id(self, clicked_button):        # saves the id of a clicked button to find the appropriate ellipse/object
        for id, button in self.buttons_ids.items():
            if button == clicked_button:
                self.picked_id = id

    # Camera footage
    def test_camera(self):         # checks if a camera of a given id is available and sets the frame resolution
        self.camswitch = False
        self.cap = cv.VideoCapture(self.cv_serialport_combobox.currentIndex(), cv.CAP_DSHOW)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 360)
        if self.cap is None or not self.cap.isOpened():
            self.cv_camstatus_label.setText("NOT LOADED")
            self.cv_camstatus_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
            self.cv_camstatus_label.setStyleSheet("QLabel { color : red }")
            self.cv_footage_label.setText(QtCore.QCoreApplication.translate("MainWindow","<html><head/><body><p><span style=\" font-size:20pt; color:#aa090e;\">NO RECORDING</span></p></body></html>"))
        else:
            self.cv_camstatus_label.setText("LOADED")
            self.cv_camstatus_label.setFont(QtGui.QFont("Ms Shell Dlg 2", 10, weight=QtGui.QFont.Bold))
            self.cv_camstatus_label.setStyleSheet("QLabel { color : green }")

    def set_frame(self, frame):           # displays an image in a pyqt GUI
        frame = cv.resize(frame, (960, 540))
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        frame = QtGui.QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QtGui.QImage.Format_RGB888)
        self.cv_footage_label.setPixmap(QtGui.QPixmap.fromImage(frame))

    def footage(self):         # functions mentioned above are implemented on live camera footage
        if self.cap is not None:
            self.camswitch = not self.camswitch
            self.detectswitch = False
            self.calibswitch = False
            self.direction_vector = (0, 0)
            iterations = 0
            while self.camswitch is True:          # video footage is shown only while the switch is ON
                success, self.frame = self.cap.read()
                if success is True:
                    iterations += 1
                    self.aruco_calibration()
                    self.object_detection()
                    if self.cv_arucosearch_radiobutton1.isChecked() or self.cv_operation_radiobutton1.isChecked():
                        self.autosearch()
                    if iterations == 5:            # the program moves the robot only each nth frame to prevent overflow
                        self.move_accordingly()
                        iterations = 0
                    self.set_frame(self.frame)
                    key = cv.waitKey(1) & 0xFF
                else:
                    self.camswitch = False
                    break
            else:
                self.cv_footage_label.setText(QtCore.QCoreApplication.translate("MainWindow","<html><head/><body><p><span style=\" font-size:20pt; color:#aa090e;\">NO RECORDING</span></p></body></html>"))

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    application = QtWidgets.QMainWindow()
    application.show()
    prog = Application(application)
    sys.exit(app.exec())
