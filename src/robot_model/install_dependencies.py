#!/usr/bin/env python3
"""
Script to install required dependencies for the robot collision checker.
Run this script first before using collision_check.py
"""

import subprocess
import sys

def install_package(package):
    """Install a package using pip"""
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

def main():
    packages = [
        "numpy",
        "pinocchio", 
        "hppfcl",
        "meshcat"
    ]
    
    print("Installing required packages for robot collision checking...")
    
    for package in packages:
        try:
            print(f"Installing {package}...")
            install_package(package)
            print(f"✓ {package} installed successfully")
        except subprocess.CalledProcessError as e:
            print(f"✗ Failed to install {package}: {e}")
            return False
    
    print("\n✓ All packages installed successfully!")
    print("You can now run: python test_collision/collision_check.py")
    return True

if __name__ == "__main__":
    main()