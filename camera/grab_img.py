import cv2
import os
import re
from pathlib import Path
from ctu_crs import CRS93

robot = CRS93()

robot.initialize(home = False)

# Grab an image from the camera
img = robot.grab_image()

if img is not None:
    print(f"✅ Successfully grabbed image with shape: {img.shape}")

    save_dir = Path(__file__).parent / "ArUco_codes"
    save_dir.mkdir(exist_ok=True)

    # Find the next available image number
    existing_files = [f.name for f in save_dir.iterdir() if f.name.startswith("image_") and f.name.endswith(".png")]
    
    if existing_files:
        # Extract numbers using regex
        nums = [int(re.search(r"image_(\d+)\.png", f).group(1)) for f in existing_files if re.search(r"image_(\d+)\.png", f)]
        next_num = max(nums) + 1
    else:
        next_num = 1

    # Format the number as 3 digits (001, 002, 003, …)
    filename = f"image_{next_num:03d}.png"
    filepath = os.path.join(save_dir, filename)

    # Save the image
    cv2.imwrite(filepath, img)
    print(f"💾 Image saved to {filepath}")

else:
    print("⚠️ No image captured. Skipping save.")

robot.release()
# robot.close()