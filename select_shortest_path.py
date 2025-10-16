#!/usr/bin/env python3
"""
Script pro výběr nejkratší cesty mezi aktuální polohou robota a možnými cílovými pozicemi.
Používá Euklidovskou vzdálenost v kloubovém prostoru (joint space).
"""

import numpy as np


def normalize_angle(angle):
    """
    Normalizuje úhel do rozsahu [-pi, pi].
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_joint_distance(q_current, q_target):
    """
    Vypočítá vzdálenost mezi dvěma konfiguracemi robota v kloubovém prostoru.
    Bere v úvahu cyklickou povahu úhlů (normalizuje rozdíly úhlů).
    
    Args:
        q_current: Aktuální konfigurace robota (6 kloubů)
        q_target: Cílová konfigurace robota (6 kloubů)
    
    Returns:
        Euklidovská vzdálenost v kloubovém prostoru
    """
    # Vypočítáme rozdíly a normalizujeme je
    diff = np.array(q_target) - np.array(q_current)
    
    # Normalizujeme rozdíly úhlů do rozsahu [-pi, pi]
    normalized_diff = np.array([normalize_angle(d) for d in diff])
    
    # Vrátíme Euklidovskou vzdálenost
    return np.linalg.norm(normalized_diff)


def find_shortest_path(q_current, q_targets):
    """
    Najde nejkratší cestu z aktuální pozice do jedné z cílových pozic.
    
    Args:
        q_current: Aktuální konfigurace robota (array 6 prvků)
        q_targets: Seznam možných cílových konfigurací (list of arrays)
    
    Returns:
        tuple: (index nejbližší konfigurace, nejbližší konfigurace, vzdálenost)
    """
    distances = []
    
    for i, q_target in enumerate(q_targets):
        dist = compute_joint_distance(q_current, q_target)
        distances.append(dist)
        print(f"Konfigurace {i}: vzdálenost = {dist:.6f} rad")
    
    # Najdeme index s minimální vzdáleností
    min_index = np.argmin(distances)
    min_distance = distances[min_index]
    best_config = q_targets[min_index]
    
    return min_index, best_config, min_distance


if __name__ == "__main__":
    # Aktuální poloha robota
    robot_get_q = np.array([-6.28318531e-05, 5.90462339e-02, -9.24774921e-01, 
                            -3.14149934e+00, 7.04941976e-01, 3.14159265e+00])
    
    # Možné cílové konfigurace (8 IK řešení)
    robot_qs = [
        np.array([-6.28318531e-05, 5.90119371e-02, -9.24691862e-01, -3.14149934e+00,
                  7.04990738e-01, -3.14159265e+00]),
        np.array([3.14152982e+00, -5.90119371e-02, 9.24691862e-01, -3.14149934e+00,
                  -7.04990738e-01, 7.02176514e-09]),
        np.array([-6.28318531e-05, 5.90119371e-02, -9.24691862e-01, 9.33092853e-05,
                  -7.04990738e-01, 7.02176562e-09]),
        np.array([3.14152982e+00, 9.05544561e-01, -9.24691862e-01, -3.14153218e+00,
                  -1.58981796e+00, 7.22233297e-05]),
        np.array([3.14152982e+00, -5.90119371e-02, 9.24691862e-01, 9.33092853e-05,
                  7.04990738e-01, -3.14159265e+00]),
        np.array([3.14152982e+00, 9.05544561e-01, -9.24691862e-01, 6.04778560e-05,
                  1.58981796e+00, -3.14152043e+00]),
        np.array([-6.28318531e-05, -9.05544561e-01, 9.24691862e-01, -3.14153218e+00,
                  1.58981796e+00, -3.14152043e+00]),
        np.array([-6.28318531e-05, -9.05544561e-01, 9.24691862e-01, 6.04778560e-05,
                  -1.58981796e+00, 7.22233297e-05])
    ]
    
    print("=" * 70)
    print("HLEDÁNÍ NEJKRATŠÍ CESTY V KLOUBOVÉM PROSTORU")
    print("=" * 70)
    print(f"\nAktuální poloha robota:")
    print(f"  q = {robot_get_q}")
    print(f"\nVypočítávám vzdálenosti ke všem možným konfiguracím...\n")
    
    # Najdeme nejkratší cestu
    best_index, best_config, min_distance = find_shortest_path(robot_get_q, robot_qs)
    
    print("\n" + "=" * 70)
    print("VÝSLEDEK")
    print("=" * 70)
    print(f"Nejbližší konfigurace: index {best_index}")
    print(f"Vzdálenost: {min_distance:.6f} rad")
    print(f"\nCílová konfigurace:")
    print(f"  q_target = {best_config}")
    print(f"\nRozdíl (normalizovaný):")
    diff = best_config - robot_get_q
    normalized_diff = np.array([normalize_angle(d) for d in diff])
    print(f"  Δq = {normalized_diff}")
    print("=" * 70)
