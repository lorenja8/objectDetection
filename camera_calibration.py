import cv2 as cv
import numpy as np

# camera footage
cap = cv.VideoCapture(1)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
keypointFrame = 5

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

index = 0

while True:
    success, img = cap.read()
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

    # If found, add object points, image points (after refining them)
    if ret == True and index == keypointFrame:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        index = 0

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7, 6), corners2, ret)
    else:
        pass

    index += 1 if index < keypointFrame else 0

    counter = len(objpoints)
    cv.putText(img, str(counter), (50, 640), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
    cv.imshow('img', img)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

ret, mtx, dist, rvec, tvec = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

np.save("calibration/ret.npy", ret)
np.save("calibration/mtx.npy", mtx)
np.save("calibration/dist.npy", dist)
np.save("calibration/rvec.npy", rvec)
np.save("calibration/tvec.npy", tvec)