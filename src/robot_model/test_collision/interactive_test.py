#!/usr/bin/env python3
"""
Interactive collision testing script.
"""

import numpy as np
from collision_check_smart import setup_collision_model_smart, compute_collision_contacts_smart, model
from pinocchio.visualize import MeshcatVisualizer

def test_configuration_interactive():
    """Interactive testing of robot configurations."""
    
    collision_model, visual_model = setup_collision_model_smart()
    
    # Setup visualizer
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    
    print("\n=== Interactive Collision Testing ===")
    print("Commands:")
    print("  'q' - quit")
    print("  'test' - test predefined configurations")
    print("  'random' - test random configuration")
    print("  'zero' - test zero configuration")
    print("  'custom' - enter custom joint values")
    
    while True:
        cmd = input("\nEnter command: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == 'test':
            test_predefined_configs(collision_model, viz)
        elif cmd == 'random':
            test_random_config(collision_model, viz)
        elif cmd == 'zero':
            test_zero_config(collision_model, viz)
        elif cmd == 'custom':
            test_custom_config(collision_model, viz)
        else:
            print("Unknown command!")

def test_predefined_configs(collision_model, viz):
    """Test predefined configurations."""
    from collision_check import q1, q2, q11
    
    configs = {
        "q1": q1,
        "q2": q2, 
        "q11": q11
    }
    
    for name, q in configs.items():
        contacts = compute_collision_contacts_smart(q, collision_model)
        status = "COLLISION" if contacts else "free"
        print(f"{name}: {status}")
        
        if contacts:
            for contact in contacts:
                print(f"  {contact['first']} ↔ {contact['second']}, penetration {contact['penetration']:.4f} m")
        
        viz.display(q)
        input(f"Showing {name} ({status}). Press Enter for next...")

def test_random_config(collision_model, viz):
    """Test random configuration within joint limits."""
    # Joint limits (approximate)
    q_min = np.array([-3.054, -1.571, -1.919, -3.142, -1.833, -3.142])
    q_max = np.array([3.054, 1.571, 1.919, 3.142, 1.833, 3.142])
    
    q_random = np.random.uniform(q_min, q_max)
    
    contacts = compute_collision_contacts_smart(q_random, collision_model)
    status = "COLLISION" if contacts else "free"
    
    print(f"Random config: {status}")
    print(f"q = {q_random}")
    
    if contacts:
        for contact in contacts:
            print(f"  {contact['first']} ↔ {contact['second']}, penetration {contact['penetration']:.4f} m")
    
    viz.display(q_random)

def test_zero_config(collision_model, viz):
    """Test zero configuration."""
    q_zero = np.zeros(6)
    
    contacts = compute_collision_contacts_smart(q_zero, collision_model)
    status = "COLLISION" if contacts else "free"
    
    print(f"Zero config: {status}")
    print(f"q = {q_zero}")
    
    if contacts:
        for contact in contacts:
            print(f"  {contact['first']} ↔ {contact['second']}, penetration {contact['penetration']:.4f} m")
    
    viz.display(q_zero)

def test_custom_config(collision_model, viz):
    """Test user-provided configuration."""
    try:
        print("Enter 6 joint values (space-separated):")
        values = input().strip().split()
        if len(values) != 6:
            print("Error: Need exactly 6 values!")
            return
            
        q_custom = np.array([float(v) for v in values])
        
        contacts = compute_collision_contacts_smart(q_custom, collision_model)
        status = "COLLISION" if contacts else "free"
        
        print(f"Custom config: {status}")
        print(f"q = {q_custom}")
        
        if contacts:
            for contact in contacts:
                print(f"  {contact['first']} ↔ {contact['second']}, penetration {contact['penetration']:.4f} m")
        
        viz.display(q_custom)
        
    except ValueError as e:
        print(f"Error parsing values: {e}")

if __name__ == "__main__":
    test_configuration_interactive()