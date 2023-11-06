import cv2 as cv
import numpy as np


class tracker:
    def __init__(self):
        self.ellipses_ids = {}
        self.ids = 0

    def update(self, detections):       # updates the dictionary of ids and ellipses
        new_ellipses_ids = {}
        for ellipse in detections:      # iterates through the list of new detections by object_detection.py
            (x, y), (_, _), _ = ellipse
            c = np.array([x, y])

            already_detected = False
            for id, ellipse0 in self.ellipses_ids.items():     # compares newly found and previously detected ellipses
                (x0, y0), (_, _), _ = ellipse0
                c0 = np.array([x0, y0])
                distance = np.linalg.norm(c-c0)

                if distance <= 25:                             # if a match is found, cycle is broken
                    already_detected = True
                    break

            if already_detected is True:                       # updates the location of a previously detected ellipse
                new_ellipses_ids[id] = ellipse

            elif already_detected is False:                    # ads newly detected ellipses into the dictionary
                new_ellipses_ids[self.ids] = ellipse
                self.ids += 1

        self.ellipses_ids = new_ellipses_ids.copy()

    def draw_ellipse(self, frame):            # draws tracked ellipses, their id and the angle of the main axis
        ellipses = self.ellipses_ids
        for key in ellipses:
            (x, y), (MA, ma), angle = ellipses[key]
            cv.ellipse(frame, (int(x), int(y)), (int(MA) // 2, int(ma) // 2), angle, 0, 360, (0, 0, 0), thickness=2)
            cv.putText(frame, str(key), (int(x), int(y - 20)), cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 0), 2)
            cv.putText(frame, str(int(angle)), (int(x), int(y + 20)), cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 0), 2)