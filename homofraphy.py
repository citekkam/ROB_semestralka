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

import os
import yaml
from typing import List, Tuple, Union

# Dark threshold for detecting circle
DARK_TRESH = 150
# SE3 from gripper to center of circle
CRC_OFF = SE3([
    [1, 0, 0, 1.35]
    [0, 1, 0, 0]
    [0, 0, 1, 0]
    [0, 0, 0, 1]
])

def load_image_yaml_pairs(folder: str = "exporty") -> Tuple[List[ArrayLike], List[dict]]:
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

    [{
        "RPY": [-0.0005572332585040621, -3.141058227474627, 0.0005185830258253442],
        "translation_vector": [0.5093259019899434, -0.17564068853313258, 0.04918733225140541]
    },
    {
        "RPY": [-0.0005572332585040621, -3.141058227474627, 0.0005185830258253442],
        "translation_vector": [0.5093569397977782, -0.08814069881074972, 0.04918733225140541]
    },
    ...
    ]
    """

    images = np.asarray(images)
    assert images.shape[0] == len(hoop_positions)
    centers = []

    for img in images:
        # todo HW03: Detect circle in each image
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img_gray = cv2.medianBlur(img_gray, 5)
        rows = img_gray.shape[0]

        img_gray = ((img_gray < DARK_TRESH) * 255).astype(np.uint8)

        # print(img_gray)
        # cv2.imshow("gray", img_gray)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        circles = cv2.HoughCircles(img_gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=300)
        

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(img, center, 1, (0, 100, 100), 3)
                # circle outline
                centers.append(center)
        
        
        # cv2.imshow("detected circles", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    # todo HW03: Find homography using cv2.findHomography. Use the hoop positions and circle centers.

    # print(hoop_positions)
    hoop_vectors = []
    for i in range(len(hoop_positions)):
        hoop_vectors.append(hoop_positions[i]["translation_vector"][0:2])

    print(centers)
    print(hoop_vectors)
    centers = np.array(centers, dtype=np.float32)
    hoop_vectors = np.array(hoop_vectors, dtype=np.float32)
    H , _ = cv2.findHomography(centers, hoop_vectors)
    return H


if __name__ == "__main__":
    imgs, hoop_pos = load_image_yaml_pairs()