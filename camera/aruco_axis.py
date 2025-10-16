# calibrate_charuco_noglob.py
import cv2, numpy as np, yaml
from pathlib import Path

# === Settings (must match your printed board) ===
DICT = cv2.aruco.DICT_4X4_50
SQUARES_X, SQUARES_Y = 5, 5
SQUARE_LEN = 0.025   # meters (25 mm)
MARKER_LEN = 0.018   # meters (18 mm)

IMG_DIR = Path(__file__).parent / "ArUco_codes"  # folder with your calibration images
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
SHOW_IMAGES = True  # Set to False to skip visualization

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
        
        # Draw detected corners on the image
        if SHOW_IMAGES:
            img_display = img.copy()
            cv2.aruco.drawDetectedMarkers(img_display, corners, ids)
            cv2.aruco.drawDetectedCornersCharuco(img_display, ch_corners, ch_ids)
            
            # Resize for better viewing if image is too large
            h, w = img_display.shape[:2]
            if w > 1280:
                scale = 1280 / w
                img_display = cv2.resize(img_display, (int(w*scale), int(h*scale)))
            
            cv2.imshow(f'ChArUco Detection - {path.name} ({n} corners)', img_display)
            print(f"Processed {path.name}: {n} ChArUco corners detected")
            key = cv2.waitKey(500)  # Wait 500ms between images (or press any key to continue)
            if key == 27:  # ESC to skip remaining images
                cv2.destroyAllWindows()
                break

# --- Check we collected data ---
if not all_corners:
    raise SystemExit("No ChArUco corners—check images, board size, and lighting.")

print(f"\n{'='*60}")
print(f"Celkem použito {len(all_corners)} obrázků pro kalibraci")
print(f"Celkem nalezeno {len(files)} obrázků ve složce")
print(f"{'='*60}\n")

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
calib_path = Path(__file__).parent / "calib.yaml"
with open(calib_path, "w") as f:
    yaml.dump(data, f)
print(f"Wrote {calib_path}")

# Close any remaining windows
cv2.destroyAllWindows()
