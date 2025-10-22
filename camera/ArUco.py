import cv2
import numpy as np

IMG_PATH = "/home/kamil/PycharmProjects/ROB_semestralka/exporty/data_1.png"
START_CORNER = 0   # 0=TL, 1=TR, 2=BR, 3=BL (OpenCV's corner order)

# --- load image ---
img = cv2.imread(IMG_PATH, cv2.IMREAD_COLOR)
if img is None:
    raise SystemExit(f"Cannot read image: {IMG_PATH}")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# --- ArUco 4x4_50 detector ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

# --- detect ---
corners, ids, _ = detector.detectMarkers(gray)

out = img.copy()
if ids is None or len(ids) == 0:
    print("No markers found.")
else:
    for c, marker_id in zip(corners, ids.flatten()):
        # c has shape (1,4,2) -> take the 4 corners in canonical order
        pts = c[0].astype(np.float32)  # [0]=TL, [1]=TR, [2]=BR, [3]=BL

        # Optionally pick a different start corner
        p0 = pts[START_CORNER]
        p1 = pts[(START_CORNER + 1) % 4]  # next clockwise
        p3 = pts[(START_CORNER + 3) % 4]  # previous (counter-clockwise)

        # Edge direction vectors (unit length)
        v1 = p1 - p0
        v3 = p3 - p0
        n1 = v1 / (np.linalg.norm(v1) + 1e-9)
        n3 = v3 / (np.linalg.norm(v3) + 1e-9)

        # Use the average side length for arrow size
        side_len = 0.5 * (np.linalg.norm(pts[1] - pts[0]) + np.linalg.norm(pts[3] - pts[0]))

        # Arrow endpoints
        a1 = (p0 + n1 * side_len).astype(int)
        a3 = (p0 + n3 * side_len).astype(int)
        p0i = tuple(p0.astype(int))

        # Draw marker, start corner, and the two vectors
        cv2.aruco.drawDetectedMarkers(out, [c], np.array([[marker_id]]))
        cv2.circle(out, p0i, 6, (0, 0, 255), -1)  # mark start corner
        cv2.arrowedLine(out, p0i, tuple(a1), (0, 255, 0), 3, tipLength=0.15)  # along edge to next corner
        cv2.arrowedLine(out, p0i, tuple(a3), (255, 0, 0), 3, tipLength=0.15)  # along edge to previous corner

        # Label
        cv2.putText(out, f"id={marker_id}", tuple(pts[2].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    print(f"Found {len(ids)} marker(s): {ids.flatten().tolist()}")

# --- save result ---
save_path = "output_vectors.jpg"
cv2.imwrite(save_path, out)
print(f"Saved: {save_path}")
