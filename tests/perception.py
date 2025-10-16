from typing import List, Tuple, Optional
from numpy.typing import ArrayLike
import numpy as np
import cv2  # noqa

def detect_hoop_convex_hull(img) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    Detekuje konvexní obal obručí a jeho střed.
    
    Returns:
        Tuple[hull, center] nebo None pokud není detekován
        - hull: konvexní obal jako numpy array
        - center: střed jako [x, y] numpy array
    """
    # Převod do HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Vytvoření masky pro oranžovou barvu
    mask = cv2.inRange(hsv, (36, 16, 0), (173, 255, 216))
    
    # Najít kontury
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # Najít největší konturu
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Vytvořit konvexní obal
    hull = cv2.convexHull(largest_contour)
    
    # Spočítat střed konvexního obalu (centroid nebo min enclosing circle)
    M = cv2.moments(hull)
    if M["m00"] != 0:
        hull_cx = int(M["m10"] / M["m00"])
        hull_cy = int(M["m01"] / M["m00"])
    else:
        (hull_cx_f, hull_cy_f), hull_r = cv2.minEnclosingCircle(hull)
        hull_cx, hull_cy = int(hull_cx_f), int(hull_cy_f)
    
    center = np.array([hull_cx, hull_cy], dtype=np.float32)
    
    return hull, center