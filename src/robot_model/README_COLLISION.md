# Robot Collision Detection

Tento projekt implementuje collision detection pro CRS A465 robotickou ruku s obstacle detection.

## Problémy které byly opraveny:

### 1. **Absolutní cesty v URDF**
- **Problém**: URDF obsahoval absolutní cesty typu `/home/david/School/rob/robot_model/meshes/...`
- **Řešení**: Změněno na relativní cesty `meshes/...`
- **Důvod**: Absolutní cesty nejsou přenosné a Pinocchio očekává relativní cesty

### 2. **Self-collision problémy** 
- **Problém**: Robot kolidoval sám se sebou (sousední linky)
- **Řešení**: Vytvořen smart collision model který testuje pouze kolize robot-vs-obstacle
- **Implementace**: `collision_check_smart.py`

### 3. **Package directory setup**
- **Problém**: Špatné nastavení `package_dirs` pro hledání mesh souborů
- **Řešení**: Nastaveno na root adresář projektu místo urdf/ adresáře

### 4. **MeshCat viewer chyby**
- **Problém**: `viz.quit()` metoda neexistuje v novějších verzích
- **Řešení**: Odstraněno volání `quit()` 

## Použití:

### Základní test:
```bash
conda activate ctu_robotics
python test_model.py
```

### Collision detection:
```bash
conda activate ctu_robotics
python test_collision/collision_check_smart.py
```

### Interaktivní testování:
```bash
conda activate ctu_robotics
python test_collision/interactive_test.py
```

### Trajectory collision testing:
```bash
conda activate ctu_robotics  
python test_collision/trajectory_collision_timing.py
```

## Struktura collision detection:

1. **Model setup** - načtení URDF a geometrií
2. **Obstacle loading** - načtení STL mesh souborů puzzle objektů
3. **Collision pair creation** - pouze robot vs obstacle páry
4. **Forward kinematics** - výpočet pozic všech segmentů
5. **Collision detection** - FCL algoritmy pro detekci průniku
6. **Result processing** - informace o kontaktech a penetraci

## Konfigurační soubory:

- `urdf/crs_a465.urdf` - Model robota (opraveny cesty)
- `meshes/` - 3D modely robotických segmentů  
- `puzzles/` - STL soubory překážek
- `test_collision/` - Collision detection skripty

## Klíčové funkce:

- `compute_collision_contacts_smart()` - hlavní collision detection
- `load_puzzle_geometry()` - načítání obstacle geometrií
- `setup_collision_model_smart()` - správné nastavení collision páru

Collision detection nyní funguje správně a detekuje pouze kolize mezi robotem a překážkami, ne self-collisions.