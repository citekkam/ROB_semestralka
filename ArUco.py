import cv2
import numpy as np

# Choose a dictionary (make sure it matches how your marker was generated)
# Common choices: DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_ARUCO_ORIGINAL, etc.
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)


img  = cv2.imread("/home/nguyexu7/Documents/ROB/ROB_semestralka/exporty/data_11.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Detect
corners, ids, rejected = detector.detectMarkers(gray)

# Draw detections
if ids is not None and len(ids) > 0:
    cv2.aruco.drawDetectedMarkers(gray, corners, ids)

cv2.imwrite("output_markers.jpg", gray)
