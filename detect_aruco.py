# aruco_on_image.py
import cv2
import numpy as np
import sys

IMG_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/nguyexu7/Documents/ROB/ROB_semestralka/exporty/data_1.png"
ARUCO_DICT = cv2.aruco.DICT_4X4_50  # change if your markers use a different dictionary

# Optional pose: set these if you have calibration & marker size
K = None  # e.g., np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=np.float32)
D = None  # e.g., np.array([k1,k2,p1,p2,k3], dtype=np.float32)
MARKER_LENGTH_M = 0.04  # only used if K and D are set

if not hasattr(cv2, "aruco"):
    raise SystemExit("Install opencv-contrib: pip install opencv-contrib-python")

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

img = cv2.imread(IMG_PATH)
if img is None:
    raise SystemExit(f"Could not read {IMG_PATH}")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
corners, ids, _ = detector.detectMarkers(gray)

out = img.copy()
if ids is not None and len(ids) > 0:
    cv2.aruco.drawDetectedMarkers(out, corners, ids)
    if K is not None and D is not None and MARKER_LENGTH_M > 0:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH_M, K, D)
        for rvec, tvec in zip(rvecs, tvecs):
            cv2.drawFrameAxes(out, K, D, rvec, tvec, MARKER_LENGTH_M * 1.5)
else:
    print("No markers found.")

cv2.imwrite("annotated.png", out)
print("Saved annotated.png")
