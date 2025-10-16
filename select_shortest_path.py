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


def compute_joint_distance(q_current, q_target, joint_weights=None):
    """
    Vypočítá vzdálenost mezi dvěma konfiguracemi robota v kloubovém prostoru.
    Bere v úvahu cyklickou povahu úhlů (normalizuje rozdíly úhlů).
    
    Args:
        q_current: Aktuální konfigurace robota (6 kloubů)
        q_target: Cílová konfigurace robota (6 kloubů)
        joint_weights: Váhy pro jednotlivé klouby (None = [1,1,1,1,1,1])
                      Vyšší váha = větší penalizace pohybu daného kloubu
    
    Returns:
        Vážená Euklidovská vzdálenost v kloubovém prostoru
    """
    # Výchozí váhy - penalizujeme klouby 3 a 5 (indexy 3 a 5)
    if joint_weights is None:
        # Klouby 3 a 5 mají vyšší váhu (10x), aby se preferovaly konfigurace
        # s menší změnou těchto kloubů
        joint_weights = np.array([1.0, 1.0, 1.0, 10.0, 1.0, 10.0])
    
    # Vypočítáme rozdíly a normalizujeme je
    diff = np.array(q_target) - np.array(q_current)
    
    # Normalizujeme rozdíly úhlů do rozsahu [-pi, pi]
    normalized_diff = np.array([normalize_angle(d) for d in diff])
    
    # Aplikujeme váhy na jednotlivé klouby
    weighted_diff = normalized_diff * joint_weights
    
    # Vrátíme váženou Euklidovskou vzdálenost
    return np.linalg.norm(weighted_diff)


def find_shortest_path(q_current, q_targets, joint_weights=None):
    """
    Najde nejkratší cestu z aktuální pozice do jedné z cílových pozic.
    
    Args:
        q_current: Aktuální konfigurace robota (array 6 prvků)
        q_targets: Seznam možných cílových konfigurací (list of arrays)
        joint_weights: Váhy pro jednotlivé klouby
    
    Returns:
        tuple: (index nejbližší konfigurace, nejbližší konfigurace, vzdálenost)
    """
    distances = []
    for i, q_target in enumerate(q_targets):
        dist = compute_joint_distance(q_current, q_target, joint_weights)
        diff = np.array(q_target) - np.array(q_current)
        normalized_diff = np.array([normalize_angle(d) for d in diff])
        distances.append((i, dist, normalized_diff))
        print(f"Konfigurace {i}: vzdálenost = {dist:.6f} rad")
        print(f"  Δq[3] = {normalized_diff[3]:+.6f} rad, Δq[5] = {normalized_diff[5]:+.6f} rad")
    # Seřadíme indexy podle vzdálenosti
    sorted_distances = sorted(distances, key=lambda x: x[1])
    return sorted_distances


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
    print(f"\nPenalizace kloubů:")
    print(f"  Kloub 3 (index 3): 10x penalizace")
    print(f"  Kloub 5 (index 5): 10x penalizace")
    print(f"\nVypočítávám vzdálenosti ke všem možným konfiguracím...\n")
    
    # Váhy pro klouby - silná penalizace pro klouby 3 a 5
    joint_weights = np.array([1.0, 1.0, 1.0, 10.0, 1.0, 10.0])
    
    # Získáme seřazené indexy podle vzdálenosti
    sorted_distances = find_shortest_path(robot_get_q, robot_qs, joint_weights)
    print("\n" + "=" * 70)
    print("SEŘAZENÉ INDEXY KONFIGURACÍ PODLE VZDÁLENOSTI")
    print("=" * 70)
    for idx, dist, normalized_diff in sorted_distances:
        print(f"Index: {idx}, Vzdálenost: {dist:.6f} rad, Δq[3]: {normalized_diff[3]:+.6f} rad, Δq[5]: {normalized_diff[5]:+.6f} rad")
    print("=" * 70)
