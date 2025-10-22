#!/usr/bin/env python3
import os, cv2, numpy as np

IMG_PATH = "/home/kamil/PycharmProjects/ROB_semestralka/exporty/data_1.png"

# --- load & gray ---
if not os.path.exists(IMG_PATH): raise FileNotFoundError(IMG_PATH)
img = cv2.imread(IMG_PATH, cv2.IMREAD_COLOR)
if img is None: raise IOError("Cannot read image.")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# --- detect ArUco 4x4_50 ---
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
if hasattr(cv2.aruco, "ArucoDetector"):
    det = cv2.aruco.ArucoDetector(dic, params)
    corners, ids, _ = det.detectMarkers(gray)
else:
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dic, parameters=params)

if ids is None or len(ids) < 2:
    raise SystemExit("Need at least 2 ArUco markers.")

# --- pick two (smallest IDs) & origin ---
ids_flat = ids.flatten()
centers = [c[0].mean(axis=0) for c in corners]
order = np.argsort(ids_flat)[:2]
pair_ids = ids_flat[order]
pair_centers = [centers[i] for i in order]
origin = (pair_centers[0] + pair_centers[1]) / 2.0

# --- reference marker for axis orientation (use first of the pair) ---
ref_idx = np.where(ids_flat == pair_ids[0])[0][0]
rc = corners[ref_idx][0].astype(np.float32)   # (4,2)
p0, p1, p3 = rc[0], rc[1], rc[3]

# raw axis vectors (along edges)
vx_raw = (p1 - p0)
vy_raw = (p3 - p0)

# normalize
vx = vx_raw / (np.linalg.norm(vx_raw) + 1e-12)
vy = vy_raw / (np.linalg.norm(vy_raw) + 1e-12)

# print normalized vectors (+ sanity lengths)
print("Normalized axis vectors (image coords):")
print(f"+X = [{vx[0]:.6f}, {vx[1]:.6f}]  | |+X| = {np.linalg.norm(vx):.6f}")
print(f"+Y = [{vy[0]:.6f}, {vy[1]:.6f}]  | |+Y| = {np.linalg.norm(vy):.6f}")

# --- draw axes (same as before) ---
edge01 = np.linalg.norm(rc[1] - rc[0]); edge03 = np.linalg.norm(rc[3] - rc[0])
L = 0.5 * 0.9 * (edge01 + edge03) * 0.5

out = img.copy()
cv2.aruco.drawDetectedMarkers(out, corners, ids)
O  = origin.astype(int)
Xp = (origin + vx * L).astype(int)
Yp = (origin + vy * L).astype(int)
cv2.circle(out, O, 6, (255,255,255), -1)
cv2.arrowedLine(out, O, Xp, (0,0,255), 3, tipLength=0.15)
cv2.arrowedLine(out, O, Yp, (0,255,0), 3, tipLength=0.15)
cv2.putText(out, "+X", tuple(Xp), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
cv2.putText(out, "+Y", tuple(Yp), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)


# --- save ---
cv2.imwrite("output_axes_aligned.jpg", out)
print("Uloženo: output_axes_aligned.jpg")
