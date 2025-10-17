import cv2
import numpy as np
import glob
from cv2 import aruco

# Parametry pro detekci
SQUARES_X, SQUARES_Y = 5, 5
SQUARE_LEN = 0.025  # velikost čtverce v metrech (25 mm)
MARKER_LEN = 0.018  # velikost markeru (18 mm)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Vytvoření ChArUco boardu (this is essential for interpolation)
board = aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, aruco_dict)

detector_params = aruco.DetectorParameters()

# Listy pro uchování dat
objpoints = []  # světové body
imgpoints = []  # obrazové body (z rohových bodů ChArUco)

# Načítání všech obrázků pomocí glob (automatically load images in the specified folder)
images = sorted(glob.glob("ArUco_codes/*.png"))

# Counter for valid frames
frame_counter = 0

# Lists to store detected corners and ids
all_corners = []  # Detected corners (marker corners)
all_ids = []  # Detected ids for each image
counter = []  # List to store the number of markers detected in each image

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"Error: Could not read image {fname}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detekce markerů
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)
    print(f"Markers detected in {fname}: {len(corners)}")  # Debugging

    # Pokud nebyly detekovány žádné markery
    if len(corners) == 0:
        continue

    # Interpolace rohových bodů ChArUco
    retval, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)  # Pass the board here
    print(f"Interpolation result for {fname}: retval = {retval}")  # Debugging

    # Print corners and IDs to debug
    print(f"ch_corners: {len(ch_corners)} corners detected")
    print(f"ch_ids: {len(ch_ids)} IDs detected")

    # Pokud nebyly detekovány rohy nebo méně než 4 body
    if retval < 4 or ch_corners is None or ch_ids is None or len(ch_ids) == 0:
        print(f"Skipping image {fname} due to insufficient valid corners or IDs.")
        continue

    # Debugging check: if corners and IDs match in length
    if len(ch_corners) != len(ch_ids):
        print(f"Warning: Mismatch in corners and IDs lengths for {fname}. Skipping image.")
        continue

    # Přidání do seznamu, pokud jsou platné rohy a ID
    objpoints.append(board.getObjPoints())  # We are using board here to fetch object points
    imgpoints.append(ch_corners)

    # Add the detected corners and ids to the lists
    all_corners.append(corners)  # Add detected corners for calibration
    all_ids.append(ids)  # Add detected ids for calibration

    # Count the number of markers detected in the current frame and append it to counter
    counter.append(len(ids))  # Count the markers detected in this image

    frame_counter += 1  # Increment the frame counter for valid frames

    # Optional: Visualize detected markers
    img_markers = aruco.drawDetectedMarkers(img, corners, ids)
    # Commented out to avoid potential memory issue
    # cv2.imshow("Detected Markers", img_markers)
    # cv2.waitKey(1)  # Wait until a key is pressed
    # cv2.destroyAllWindows()

# Zkontrolujte, že máme alespoň nějaké platné body pro kalibraci
if len(objpoints) > 0 and len(imgpoints) > 0:
    print(f"Total valid calibration images: {frame_counter}")  # Display the frame counter

    # Kalibrace kamery s ArUco boardem (using multiple frames)
    image_size = gray.shape[::-1]
    try:
        # Use the detected corners and object points for camera calibration
        ret, K, distCoeffs, rvecs, tvecs = aruco.calibrateCameraAruco(all_corners, all_ids, counter, board, image_size, None, None)

        print(f"RMS reprojekční chyba: {ret}")
        print(f"K (kamera):\n{K}")
        print(f"distCoeffs (distorzní koeficienty):\n{distCoeffs}")
        print(f"Rotation Vectors (rvecs):\n{rvecs}")
        print(f"Translation Vectors (tvecs):\n{tvecs}")

        # Ověření pomocí undistort
        img0 = cv2.imread(images[0])
        und = cv2.undistort(img0, K, distCoeffs)
        cv2.imwrite("undistorted_image.png", und)
    except Exception as e:
        print(f"Error during calibration: {e}")
else:
    print("Chybí platné body pro kalibraci.")
