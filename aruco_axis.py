# calibrate_charuco_noglob.py
import cv2, numpy as np, yaml
from pathlib import Path

# === Settings (must match your printed board) ===
DICT = cv2.aruco.DICT_5X5_250
SQUARES_X, SQUARES_Y = 5, 5
SQUARE_LEN = 0.025   # meters (25 mm)
MARKER_LEN = 0.018   # meters (18 mm)

IMG_DIR = Path("ArUco_codes")          # folder with your calibration images
EXTS = {".png", ".jpg", ".jpeg"}       # accepted extensions

# --- Build file list without glob ---
files = sorted([p for p in IMG_DIR.iterdir() if p.is_file() and p.suffix.lower() in EXTS],
               key=lambda p: p.name)
if not files:
    raise SystemExit(f"No images found in {IMG_DIR} with extensions {sorted(EXTS)}")

# --- ArUco / ChArUco setup ---
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT)
board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, aruco_dict)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

all_corners, all_ids = [], []
img_size = None

for path in files:
    img = cv2.imread(str(path))
    if img is None:
        print(f"Warning: could not read {path}, skipping.")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        continue

    n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if n is not None and n >= 10:
        all_corners.append(ch_corners)
        all_ids.append(ch_ids)
        img_size = gray.shape[::-1]

# --- Check we collected data ---
if not all_corners:
    raise SystemExit("No ChArUco corners—check images, board size, and lighting.")

# --- Calibrate ---
rms, K, D, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    charucoCorners=all_corners,
    charucoIds=all_ids,
    board=board,
    imageSize=img_size,
    cameraMatrix=None,
    distCoeffs=None
)

print("RMS reprojection error:", rms)
print("K:\n", K)
print("D:\n", D.ravel())

# --- Save calibration to YAML ---
data = {
    "camera_matrix": {"rows": 3, "cols": 3, "dt": "d", "data": K.flatten().tolist()},
    "dist_coeffs":   {"rows": 1, "cols": len(D), "dt": "d", "data": D.flatten().tolist()},
}
with open("calib.yaml", "w") as f:
    yaml.dump(data, f)
print("Wrote calib.yaml")
