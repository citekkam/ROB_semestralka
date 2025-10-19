"""
Praktická ukázka kontroly limitů v IKT6 Python
"""

import numpy as np
import sys
import os

# Add parent directories to path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
grandparent_dir = os.path.dirname(parent_dir)
sys.path.insert(0, grandparent_dir)

from ikt6_python import ikt6_robot_init, ikt6_dkt, ikt6_ikt


def main():
    print("=" * 80)
    print("Ukázka kontroly limitů v IKT6 Python")
    print("=" * 80)
    
    # 1. Robot s normálními limity
    print("\n### 1. Robot s normálními limity ###\n")
    
    lengths = np.array([440, 0, 305, 0, 330, 211.0])
    offsets = np.array([0, 0, 0, 0, 0, 0.0])
    directions = np.array([1, -1, -1, 1, -1, 1.0])
    limits_max_normal = np.deg2rad([175, 90, 110, 180, 105, 180])
    limits_min_normal = np.deg2rad([-175, -90, -110, -180, -105, -180])
    
    robot_normal = ikt6_robot_init(
        name="CRS93_normal",
        lengths=lengths,
        offsets=offsets,
        directions=directions,
        limits_max=limits_max_normal,
        limits_min=limits_min_normal
    )
    
    print("Limity robotu (ve stupních):")
    print(f"  Max: {np.rad2deg(robot_normal.limits_max)}")
    print(f"  Min: {np.rad2deg(robot_normal.limits_min)}")
    
    # Test IK
    P_test = np.array([400, 200, 500, 0, np.pi/6, 0])
    J_sols = ikt6_ikt(robot_normal, P=P_test)
    
    n_valid = sum(1 for i in range(8) if not np.any(np.isnan(J_sols[:, i])))
    print(f"\nCílová poze: {P_test}")
    print(f"Počet platných IK řešení: {n_valid}/8")
    
    # Zobraz platná řešení
    for i in range(8):
        if not np.any(np.isnan(J_sols[:, i])):
            print(f"  Řešení {i+1}: {np.rad2deg(J_sols[:, i])} [stupně]")
    
    # 2. Robot s omezenými limity
    print("\n\n### 2. Robot s omezenými limity ###\n")
    
    limits_max_limited = np.deg2rad([45, 45, 45, 90, 45, 90])  # Hodně omezené!
    limits_min_limited = np.deg2rad([-45, -45, -45, -90, -45, -90])
    
    robot_limited = ikt6_robot_init(
        name="CRS93_limited",
        lengths=lengths,
        offsets=offsets,
        directions=directions,
        limits_max=limits_max_limited,
        limits_min=limits_min_limited
    )
    
    print("Limity robotu (ve stupních):")
    print(f"  Max: {np.rad2deg(robot_limited.limits_max)}")
    print(f"  Min: {np.rad2deg(robot_limited.limits_min)}")
    
    # Test stejnou pozici
    J_sols_limited = ikt6_ikt(robot_limited, P=P_test)
    
    n_valid_limited = sum(1 for i in range(8) if not np.any(np.isnan(J_sols_limited[:, i])))
    print(f"\nCílová poze: {P_test}")
    print(f"Počet platných IK řešení: {n_valid_limited}/8")
    
    if n_valid_limited > 0:
        for i in range(8):
            if not np.any(np.isnan(J_sols_limited[:, i])):
                print(f"  Řešení {i+1}: {np.rad2deg(J_sols_limited[:, i])} [stupně]")
    else:
        print("  ❌ Žádná platná řešení - pozice je nedosažitelná s těmito limity!")
    
    # 3. Ukázka kontroly jednotlivých řešení
    print("\n\n### 3. Detailní kontrola řešení ###\n")
    
    if n_valid > 0:
        # Vezmi první platné řešení
        for i in range(8):
            if not np.any(np.isnan(J_sols[:, i])):
                J_first = J_sols[:, i]
                break
        
        print(f"První platné řešení: {np.rad2deg(J_first)} [stupně]")
        print("\nKontrola limitů pro každý kloub:")
        
        for j in range(6):
            min_deg = np.rad2deg(robot_normal.limits_min[j])
            max_deg = np.rad2deg(robot_normal.limits_max[j])
            val_deg = np.rad2deg(J_first[j])
            
            in_limits = robot_normal.limits_min[j] < J_first[j] < robot_normal.limits_max[j]
            status = "✓" if in_limits else "✗"
            
            print(f"  Kloub {j+1}: {val_deg:7.2f}° [{min_deg:7.2f}° až {max_deg:6.2f}°] {status}")
    
    # 4. Test s extrémní pozicí
    print("\n\n### 4. Test s extrémní pozicí ###\n")
    
    P_extreme = np.array([1000, 500, 800, 0, 0, 0])  # Velmi daleko
    print(f"Extrémní cílová poze: {P_extreme}")
    
    J_sols_extreme = ikt6_ikt(robot_normal, P=P_extreme)
    n_valid_extreme = sum(1 for i in range(8) if not np.any(np.isnan(J_sols_extreme[:, i])))
    
    print(f"Počet platných IK řešení: {n_valid_extreme}/8")
    
    if n_valid_extreme == 0:
        print("  ❌ Pozice je mimo dosah robotu!")
        print("  💡 Důvod: Buď geometricky nedosažitelná, nebo všechna řešení")
        print("     porušují limity kloubů.")
    
    # 5. Srovnání normálních vs omezených limitů
    print("\n\n### 5. Shrnutí: Vliv limitů na IK ###\n")
    
    test_positions = [
        np.array([300, 100, 400, 0, 0, 0]),
        np.array([400, 200, 500, 0, np.pi/6, 0]),
        np.array([500, 300, 600, 0, np.pi/4, 0]),
        np.array([600, 200, 400, 0, -np.pi/6, 0]),
    ]
    
    print("Pozice                              Normální    Omezené")
    print("-" * 60)
    
    for pos in test_positions:
        J_normal = ikt6_ikt(robot_normal, P=pos)
        J_limited = ikt6_ikt(robot_limited, P=pos)
        
        n_normal = sum(1 for i in range(8) if not np.any(np.isnan(J_normal[:, i])))
        n_limited = sum(1 for i in range(8) if not np.any(np.isnan(J_limited[:, i])))
        
        print(f"[{pos[0]:4.0f}, {pos[1]:4.0f}, {pos[2]:4.0f}, ...]      {n_normal}/8         {n_limited}/8")
    
    print("\n" + "=" * 80)
    print("Závěr:")
    print("  • IK automaticky filtruje řešení podle limitů")
    print("  • Užší limity = méně platných řešení")
    print("  • Řešení mimo limity jsou označena jako NaN")
    print("  • Vždy kontroluj počet platných řešení před použitím!")
    print("=" * 80)


if __name__ == "__main__":
    main()
