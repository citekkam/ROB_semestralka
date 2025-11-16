
#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2025-09-21
#     Author: Martin Cífka <martin.cifka@cvut.cz>
#
from typing import List
from numpy.typing import ArrayLike
import numpy as np
import cv2  # noqa
from PIL import Image
from se3 import SE3
from so3 import SO3

import os
import yaml
from typing import List, Tuple, Union


def hom2se3(T : ArrayLike) -> SE3:
    R = SO3([
        [T[0, 0], T[0, 1], T[0, 2]],
        [T[1, 0], T[1, 1], T[1, 2]],
        [T[2, 0], T[2, 1], T[2, 2]]
    ])
    t = np.array([
        T[0, 3], T[1, 3], T[2, 3]
    ])
    return SE3(rotation= R, translation = t)

# Dark threshold for detecting circle
DARK_TRESH = 120
# SE3 from gripper to center of circle
CRC_OFF = hom2se3(np.array([
    [1, 0, 0, -0.135],
    [0, 1, 0, 0],
    [0, 0, 1, 0.0166],
    [0, 0, 0, 1]
]))

def load_image_yaml_pairs(folder: str = "/home/muflonn/fel/semestr5/rob/semesralka/exporty") -> Tuple[List[ArrayLike], List[dict]]:
    """
    Load corresponding PNG images and YAML files from a folder.

    Expects pairs like:
        data_1.png, data_1.yaml
        data_2.png, data_2.yaml
        ...

    Args:
        folder: Path to the folder containing .png and .yaml files.

    Returns:
        images: List of image arrays (ArrayLike, BGR order).
        yaml_data: List of parsed YAML dictionaries.
    """
    # --- Collect filenames and sort numerically ---
    image_files = sorted(
        [f for f in os.listdir(folder) if f.endswith(".png")],
        key=lambda x: int(os.path.splitext(x)[0].split('_')[-1])
    )
    yaml_files = sorted(
        [f for f in os.listdir(folder) if f.endswith(".yaml")],
        key=lambda x: int(os.path.splitext(x)[0].split('_')[-1])
    )

    if len(image_files) != len(yaml_files):
        raise ValueError(
            f"Number of images ({len(image_files)}) and YAMLs ({len(yaml_files)}) do not match!"
        )

    images: List[ArrayLike] = []
    yaml_data: List[dict] = []

    # --- Load files ---
    for img_file, yaml_file in zip(image_files, yaml_files):
        img_path = os.path.join(folder, img_file)
        yaml_path = os.path.join(folder, yaml_file)

        # Read image
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(f"Could not read image: {img_path}")
        images.append(img)

        # Read YAML
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        yaml_data.append(data)

    return images, yaml_data



def correct_eff_pos(T_B2G: SE3):
    """
    Correct transformation of end effector. For homography is neccesarrry to 
    have transforamtion from base to center of the circle. 
    """
    return T_B2G * CRC_OFF


def find_hoop_homography(images: ArrayLike, hoop_positions: List[dict]) -> np.ndarray:
    """
    Find homography based on images containing the hoop and the hoop positions loaded from
    the hoop_positions.json file in the following format:
    """

    images = np.asarray(images)
    assert images.shape[0] == len(hoop_positions)
    centers = []

    hoop_vectors = []
    for i in range(len(hoop_positions)):
        # trans = hom2se3(np.array(pos["transformacni_matic"]))
        trans =  hom2se3(hoop_positions[i])
        print(trans)
        trans = trans * CRC_OFF
        print(trans)
        hoop_vectors.append(trans.translation[:2])
        print(trans.translation[:2])
    hoop_vectors = np.array(hoop_vectors, dtype=np.float32) 


    for i in range(len(images)):
        img = images[i]
        img_gray_aruco = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        img_gray = cv2.medianBlur(img_gray_aruco, 5)
        rows = img_gray.shape[0]

        img_gray = ((img_gray < DARK_TRESH) * 255).astype(np.uint8)

        # print(img_gray)
        # cv2.imshow("gray", img_gray)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        circles = cv2.HoughCircles(img_gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=120, param2=33,
                                minRadius=20, maxRadius=300)
        

        if circles is not None and len(circles[0, :]) == 1:
            circles = np.uint16(np.around(circles))
            for j in circles[0, :]:
                center = (j[0], j[1])
                # circle center
                cv2.circle(img, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = j[2]
                cv2.circle(img, center, radius, (255, 0, 255), 3)
                centers.append(center)
        else:
            hoop_vectors = np.delete(hoop_vectors, i, axis=0)
            print(f"None or more than one circle detected in image {i}!")

        # show_img = cv2.resize(img_gray, (1200, 800))
        # show_img = cv2.resize(img, (1200, 800))
        # cv2.imshow(f"detected circles {i}", show_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()


    # print(hoop_positions)

    centers = np.array(centers, dtype=np.float32)
    print(centers)
    print(* hoop_vectors)

    print(len(centers))
    print(len(hoop_vectors))
    
    H , _ = cv2.findHomography(centers, hoop_vectors)

    # saving the homography matrix
    np.save("H_matrix.npy", H)
    return H

def get_H() -> np.ndarray:
    H = np.load("H_matrix.npy")
    return H

def find_aruco(img):
    """
    Find aruco markers in the image and return their ids and corners.
    """
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    corners, ids, rejected = detector.detectMarkers(img_gray)

    cv2.aruco.drawDetectedMarkers(img, corners, ids)


    return ids, corners


def get_aruco_center(corners : ArrayLike, img = None) -> List[np.ndarray]:
    """
    Get the center position of the aruco markers using the homography H.
    """
    positions = []
    for c in corners:
        c = c[0]
        center = np.array([(c[0][0] + c[2][0]) / 2, (c[0][1] + c[2][1]) / 2, 1])
        if not img is None:
            cv2.circle(img, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
        print("center",center)

        positions.append(center)


    return positions

def get_puzzle_base(aruco_ids: List[int], aruco_corners: List[np.ndarray], H : np.ndarray ,img = None):
    """
    Returns SE3 of puzzle base
    """

    if aruco_ids is None:
        raise ValueError("At least one ArUco marker is required to determine the puzzle base.")
    elif len(aruco_ids) == 1:
        center = get_aruco_center(aruco_corners, img)[0]
        print("pos", center)
        diagonal = [-0.0375, (0.0375), 0]
        trans = H @ np.array([center[0], center[1], 1])
        trans /= trans[2]
        trans[2] = 0.055
        R = get_base_rotation(aruco_corners, H)
        
        ## translation of center
        if aruco_ids[0] == 1:
            trans = trans - R.act(diagonal)
        elif aruco_ids[0] == 2:
            trans = trans + R.act(diagonal)

        T = SE3(trans, R)

    elif len(aruco_ids) == 2:
        # Getting center
        pos1, pos2 = get_aruco_center(aruco_corners, img)

        center = (pos1 + pos2) / 2.0
        if not img is None:
            cv2.circle(img, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
        trans = H @ np.array([center[0], center[1], 1])
        trans /= trans[2]
        # Setting the z coordinate to be above the table for desired height in meters
        trans[2] = 0.055
        R = get_base_rotation(aruco_corners, H)
        T = SE3(trans, R)
    else:
        raise NotImplementedError("False positives detected, more than two ArUco markers found.")

    return T


def get_base_rotation(aruco_corners: List[np.ndarray], H : np.ndarray):
    vecs = np.zeros((len(aruco_corners), 2))
    base_x = np.array([1.0, 0.0])
    angles = []
    for c in aruco_corners:
        c0 = H @ np.append(c[0][0], 1)
        c1 = H @ np.append(c[0][1], 1)
        c0 = (c0/c0[2])[0:2]
        c1 = (c1/c1[2])[0:2]

        puzzle_x = c1 - c0
        puzzle_x = puzzle_x / np.linalg.norm(puzzle_x)

        # There is no divison by norms, because both are unit vectors
        angle = np.arccos(np.clip(np.dot(base_x, puzzle_x), -1.0, 1.0))
        if np.cross(base_x, puzzle_x) < 0:
            angle = -angle
        angles.append(angle)
        
    
    mean_angle = np.mean(angles)
    R = np.array([
        [np.cos(mean_angle), -np.sin(mean_angle), 0],
        [np.sin(mean_angle), np.cos(mean_angle), 0],
        [0, 0, 1]
    ])

    return SO3(R)
        

def get_base_T(base_pos: np.ndarray, orientation: SO3) -> np.ndarray:
    # CRC_OFF * trans
    print(orientation, np.array([base_pos[0], base_pos[1], 0.05]))
    T = SE3(rotation = orientation, translation = np.array([base_pos[0], base_pos[1], 0.05]))
    return T

def homography_check(img: ArrayLike, H: np.ndarray) -> np.ndarray: 
    centers = []
    img_gray_aruco = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    img_gray = cv2.medianBlur(img_gray_aruco, 5)
    rows = img_gray.shape[0]

    img_gray = ((img_gray < DARK_TRESH) * 255).astype(np.uint8)

    # print(img_gray)
    # cv2.imshow("gray", img_gray)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()



    circles = cv2.HoughCircles(img_gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                        param1=120, param2=33,
                        minRadius=20, maxRadius=300)


    if circles is not None and len(circles[0, :]) == 1:
        circles = np.uint16(np.around(circles))
        for j in circles[0, :]:
            center = (j[0], j[1])
            # circle center
            cv2.circle(img, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = j[2]
            cv2.circle(img, center, radius, (255, 0, 255), 3)
            centers.append(center)

    world_pos = H @ np.array([centers[0][0], centers[0][1], 1])
    return world_pos / world_pos[2]



if __name__ == "__main__":
    print(CRC_OFF)
    imgs, hoop_pos = load_image_yaml_pairs()

    # print(imgs, hoop_pos)
    H = find_hoop_homography(imgs, hoop_pos)
    print(H)
    img = imgs[4]
    # np.save("H_matrix.npy", H)
    # H_load = np.load("H_matrix.npy")
    # print("Loaded H: ", H_load)

    ids, corners = find_aruco(img)
    print(ids, corners)


    positions = get_aruco_center(corners, img)
    print(positions)

    puzzle_base = get_puzzle_base(ids, corners, H, img)


    print("Position of puzzle base: ", puzzle_base)
    trans = hom2se3(np.array(hoop_pos[4]["transformacni_matic"]))
    print("Forward kinematics: ",trans)
    H_check = homography_check(img, H)
    print("Homography check: ", H_check)

    show_img = cv2.resize(img, (1200, 800))
    cv2.imshow(f"ArUco", show_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()