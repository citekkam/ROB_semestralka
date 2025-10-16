#!/usr/bin/env python3
"""
Porovnání původního a undistorted obrázku pro vizualizaci efektu kalibrace.
"""

import cv2
import numpy as np
import yaml
from pathlib import Path

# Načteme kalibrační parametry
calib_path = Path(__file__).parent / "calib.yaml"
with open(calib_path, "r") as f:
    calib_data = yaml.safe_load(f)

K = np.array(calib_data["camera_matrix"]["data"]).reshape(3, 3)
D = np.array(calib_data["dist_coeffs"]["data"])

print("Camera Matrix (K):")
print(K)
print("\nDistortion Coefficients (D):")
print(D)
print(f"\nRadial distortion: k1={D[0]:.6f}, k2={D[1]:.6f}")
if len(D) >= 5:
    print(f"Tangential distortion: p1={D[2]:.6f}, p2={D[3]:.6f}")
    print(f"Higher order radial: k3={D[4]:.6f}")

# Najdeme testovací obrázek
IMG_DIR = Path(__file__).parent / "ArUco_codes"
EXTS = {".png", ".jpg", ".jpeg"}
files = sorted([p for p in IMG_DIR.iterdir() if p.is_file() and p.suffix.lower() in EXTS])

if not files:
    raise SystemExit(f"No images found in {IMG_DIR}")

# Použijeme první obrázek jako ukázku
test_img_path = files[0]
print(f"\nPoužívám testovací obrázek: {test_img_path.name}")

# Načteme obrázek
img = cv2.imread(str(test_img_path))
h, w = img.shape[:2]

# Vypočítáme novou kameru matrix pro undistortion
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

# Undistort obrázek
undistorted = cv2.undistort(img, K, D, None, newcameramtx)

# Ořežeme podle ROI (volitelné)
x, y, w_roi, h_roi = roi
undistorted_cropped = undistorted[y:y+h_roi, x:x+w_roi]

# Vytvoříme rozdílový obrázek pro vizualizaci změn
diff = cv2.absdiff(img, undistorted)
diff_enhanced = cv2.convertScaleAbs(diff, alpha=5)  # Zvýrazníme rozdíly

# Vypočítáme statistiky rozdílů
diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
mean_diff = np.mean(diff_gray)
max_diff = np.max(diff_gray)
pixels_changed = np.sum(diff_gray > 5)  # Pixely s rozdílem > 5
total_pixels = diff_gray.size
percent_changed = (pixels_changed / total_pixels) * 100

print("\n" + "=" * 60)
print("ANALÝZA DISTORZE")
print("=" * 60)
print(f"Průměrný rozdíl pixelů: {mean_diff:.2f}")
print(f"Maximální rozdíl: {max_diff}")
print(f"Pixelů se změnilo (>5): {pixels_changed} ({percent_changed:.2f}%)")
print(f"ROI po undistortion: {roi}")
print("=" * 60)

# Vytvoříme porovnání side-by-side
h_display, w_display = img.shape[:2]
max_width = 1920
if w_display * 2 > max_width:
    scale = max_width / (w_display * 2)
    w_display = int(w_display * scale)
    h_display = int(h_display * scale)
    img_resized = cv2.resize(img, (w_display, h_display))
    undistorted_resized = cv2.resize(undistorted, (w_display, h_display))
    diff_resized = cv2.resize(diff_enhanced, (w_display, h_display))
else:
    img_resized = img
    undistorted_resized = undistorted
    diff_resized = diff_enhanced

# Side-by-side porovnání
comparison = np.hstack([img_resized, undistorted_resized])

# Přidáme text
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(comparison, "ORIGINAL", (10, 30), font, 1, (0, 255, 0), 2)
cv2.putText(comparison, "UNDISTORTED", (w_display + 10, 30), font, 1, (0, 255, 0), 2)

# Zobrazíme výsledky
cv2.imshow("Original vs Undistorted", comparison)
cv2.imshow("Difference (Enhanced 5x)", diff_resized)

# print("\nStiskněte libovolnou klávesu pro uzavření oken...")
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Uložíme výsledky
output_dir = Path(__file__).parent / "distortion_analysis"
output_dir.mkdir(exist_ok=True)

cv2.imwrite(str(output_dir / "original.png"), img)
cv2.imwrite(str(output_dir / "undistorted.png"), undistorted)
cv2.imwrite(str(output_dir / "undistorted_cropped.png"), undistorted_cropped)
cv2.imwrite(str(output_dir / "comparison.png"), comparison)
cv2.imwrite(str(output_dir / "difference.png"), diff_enhanced)

print(f"\nVýsledky uloženy do složky: {output_dir}/")
