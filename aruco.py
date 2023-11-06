import math

import cv2 as cv
import cv2.aruco as aruco
import numpy as np


class calibrator:
    def __init__(self):
        self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        self.marker_size = 150

        # # camera calibration matrices (camera calibrated at 640x360)
        # self.camMtx0 = np.load("calibration/mtx.npy")
        # self.width = 640           # cap.get(cv.CAP_PROP_FRAME_WIDTH)
        # ratio = 640 / self.width
        # self.camMtx = np.true_divide(self.camMtx0, ratio)

        self.camMtx = np.load("calibration/mtx.npy")
        self.distCoeff = np.ndarray([0])     # np.load("calibration/dist.npy")
        self.camRvec = np.load("calibration/rvec.npy")
        self.camTvec = np.load("calibration/tvec.npy")

        self.markers = []
        self.ids = []
        self.pose_single = []
        self.angle_d = float

    def detect_markers(self, frame):            # detects markers and put them in ascending order by id
        markers, ids, _ = aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        detected = len(markers)

        if ids is None:
            ids = []

        for i in range(detected):
            x = int(markers[i][0][0][0])
            y = int(markers[i][0][0][1])
            cv.putText(frame, str(ids[i]), (x, y), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

        self.markers = markers
        self.ids = ids

    def reference_marker(self, reference_id):       # returns marker of a specific id
        ids = self.ids
        markers = self.markers
        success = False
        reference_marker = []
        for id in ids:
            if id[0] == reference_id:
                reference_marker = markers[reference_id]
                success = True
        return success, reference_marker

    def find_pose_single(self, frame, markers):      # returns the position of the aruco marker using aruco methods
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(markers, self.marker_size, self.camMtx, self.distCoeff)
        for rvec, tvec in zip(rvecs, tvecs):
            aruco.drawAxis(frame, self.camMtx, self.distCoeff, rvec, tvec, 60)

        self.pose_single = [rvecs, tvecs]

    def camera_angle(self):                      # returns the angle of the camera in respect to the marker
        rvecs, _ = self.pose_single
        rvecs = np.array(rvecs[0][0])

        rot, _ = cv.Rodrigues(rvecs)     # solve rodrigues
        k = np.array([0, 0, 1])          # unit vector in z
        k_rot = rot.dot(k)

        dot_product = np.dot(k, k_rot)
        angle_r = math.pi - np.arccos(dot_product)
        angle_d = np.degrees(angle_r)
        self.angle_d = angle_d