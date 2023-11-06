import cv2 as cv
import numpy as np

class detector:
    def __init__(self):
        self.parameters = ()
        self.area = (900, 40000)
        self.approxes = []
        self.ellipses = []

    def detect_contours(self, processed):     # finds contours, creates approximations and bounding ellipses
        area_min, area_max = self.area
        approxes = []
        ellipses = []
        contours, hierarchy = cv.findContours(processed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:                 # iterates through the list of found contours
            area = cv.contourArea(cnt)
            if (area > area_min) and (area < area_max):        # accepts contours of a certain size only
                peri = cv.arcLength(cnt, True)
                approx = cv.approxPolyDP(cnt, 0.02*peri, True)
                (x, y), (MA, ma), angle = cv.fitEllipse(cnt)

                approxes.append([approx])
                ellipses.append([(x, y), (MA, ma), angle])
            else:
                pass
        self.approxes = approxes
        self.ellipses = ellipses

###### Methods for highlighting the object #######

    def method_1(self, frame):         # black and white threshold from colored threshold
        t = self.parameters
        _, threshold_color = cv.threshold(frame, t[0], t[1], cv.THRESH_BINARY)
        gray = cv.cvtColor(threshold_color, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 5)
        _, threshold_bw = cv.threshold(blur, t[2], t[3], cv.THRESH_BINARY)
        return gray, threshold_bw

    def method_2(self, frame):         # Canny
        t = self.parameters
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (3, 3), 5)
        canny = cv.Canny(blur, t[0], t[1])
        return canny

    def method_3(self, frame):         # Sobel filter
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        scale = 1
        delta = 0
        ddepth = cv.CV_16S
        grad_x = cv.Sobel(gray, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
        grad_y = cv.Sobel(gray, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
        abs_grad_x = cv.convertScaleAbs(grad_x)
        abs_grad_y = cv.convertScaleAbs(grad_y)
        grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
        return grad

    def method_4(self, frame):          # highlights white
        t = self.parameters
        blur = cv.GaussianBlur(frame, (5, 5), 5)
        hls = cv.cvtColor(blur, cv.COLOR_BGR2HLS)

        h_min = 0
        h_max = 179
        l_min = t[0]
        l_max = t[1]
        s_min = 0
        s_max = 255

        lower = np.array([h_min, l_min, s_min])
        upper = np.array([h_max, l_max, s_max])
        mask = cv.inRange(hls, lower, upper)
        # kuka = cv.bitwise_and(frame, frame, mask=mask)
        return mask  #, kuka

    def method_5(self, frame, ratio):         # K-means
        t = self.parameters
        h = len(frame)
        w = len(frame[0])
        h_r = int(h/ratio)
        w_r = int(w/ratio)
        shrinked = cv.resize(frame, (w_r, h_r), 1, 1)
        pixel_vector = shrinked.reshape((-1, 3))
        # convert to np.float32
        pixel_vector = np.float32(pixel_vector)
        # define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 6
        ret, label, center = cv.kmeans(pixel_vector, K, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)
        # convert back into uint8, and make original image
        center = np.uint8(center)
        result = center[label.flatten()]
        recreated = result.reshape(shrinked.shape)
        resized = cv.resize(recreated, (w, h), 1, 1)
        gray = cv.cvtColor(resized, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 5)
        _, threshold = cv.threshold(blur, t[0], t[1], cv.THRESH_BINARY)
        return threshold