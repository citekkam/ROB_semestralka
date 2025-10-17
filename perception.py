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
import os
import yaml


def load_translations(folder="exporty", prefix="data_", start=1, end=30):
    rows = []
    for i in range(start, end + 1):
        path = os.path.join(folder, f"{prefix}{i}.yaml")
        if not os.path.exists(path):
            print(f"Chybí soubor: {path}")
            continue

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        T = np.array(data["transformacni_matic"], dtype=float)  # očekává se 4x4
        rows.append([T[0, 3], T[1, 3], T[2, 3]])  # prvky 1,4; 2,4; 3,4

    return np.asarray(rows, dtype=float)  # tvar (N, 3)

def _to_h(p):
    """-> homogenní [x,y,1]."""
    p = np.asarray(p, float)
    if p.ndim == 1:
        p = p[None, :]
    ones = np.ones((p.shape[0], 1))
    return np.hstack([p[:, :2], ones])

def reprojection_errors(H, img_pts, world_xy):
    """
    H: 3x3 homografie image->world
    img_pts: Nx2 pixely
    world_xy: Nx2 metry (XY)
    Vrací (rmse_world_m, rmse_img_px)
    """
    img_pts = np.asarray(img_pts, float).reshape(-1, 2)
    world_xy = np.asarray(world_xy, float).reshape(-1, 2)

    # image -> world
    P_img_h = _to_h(img_pts)  # Nx3
    P_w_h = (H @ P_img_h.T).T  # Nx3
    P_w = P_w_h[:, :2] / P_w_h[:, 2:3]  # Nx2
    err_w = np.linalg.norm(P_w - world_xy, axis=1)  # metry
    rmse_world = float(np.sqrt(np.mean(err_w ** 2)))

    # world -> image (přes H^{-1})
    Hinv = np.linalg.inv(H)
    P_w_h2 = _to_h(world_xy)
    P_i_h = (Hinv @ P_w_h2.T).T
    P_i = P_i_h[:, :2] / P_i_h[:, 2:3]
    err_i = np.linalg.norm(P_i - img_pts, axis=1)  # pixely
    rmse_img = float(np.sqrt(np.mean(err_i ** 2)))

    return rmse_world, rmse_img, err_w, err_i

def print_report(H, img_pts, world_xy, name="Homografie"):
    rmse_w, rmse_i, e_w, e_i = reprojection_errors(H, img_pts, world_xy)
    print(f"=== {name} ===")
    print(f"RMSE (image->world): {rmse_w:.6f} m")
    print(f"RMSE (world->image): {rmse_i:.3f} px")
    print(f"max err world: {e_w.max():.6f} m | max err image: {e_i.max():.2f} px")

def find_hoop_homography(hoop_positions):
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

    img_pts = []
    world_xy = []

    folder = "exporty"
    prefix = "data_"
    ext = ".png"

    for i in range(1,31):
        pose = hoop_positions[i-1]
        path = os.path.join(folder, f"{prefix}{i}{ext}")
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=120,
            param1=110,
            param2=32,
            minRadius=40,
            maxRadius=140
        )

        if circles is None:
            print("No circles found.")
            continue

        circles = np.around(circles).astype(int)
        if circles.ndim == 3:
            circles = circles[0]
        if circles.ndim == 1:
            circles = circles[np.newaxis, :]

        H, W = gray.shape[:2]
        scores = []
        for (x, y, r) in circles:
            s = float(r)
            s += 0.002 * (W - x) + 0.002 * (y)
            scores.append(s)

        best_idx = int(np.argmax(scores))
        x, y, r = circles[best_idx]

    # todo HW03: Find homography using cv2.findHomography. Use the hoop positions and circle centers.


        X, Y = pose[:2]
        img_pts.append([x, y])
        world_xy.append([X, Y])

    src = np.asarray(img_pts).reshape(-1, 1, 2)
    dst = np.asarray(world_xy).reshape(-1, 1, 2)

    Homograph, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransacReprojThreshold=0.003)


    img_pts_2 = np.asarray(img_pts, float).reshape(-1, 2)
    world_xy_2 = np.asarray(world_xy, float).reshape(-1, 2)
    print_report(Homograph, img_pts_2, world_xy_2)

    return Homograph

if __name__ == "__main__":
    translations = load_translations("exporty", "data_", 1, 30)
    print(find_hoop_homography(translations))
