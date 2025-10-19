# Jak IKT6 Python Kontroluje Limity Robotu

## 🎯 Přehled

Knihovna IKT6 kontroluje limity robotu na **dvou úrovních**:

1. **Během IK výpočtu** - filtruje neplatná řešení
2. **V testovací funkci** - kontroluje náhodně generované konfigurace

---

## 📐 1. Definice Limitů

### Při inicializaci robota:

```python
# Příklad: CRS93
limits_max = np.deg2rad([175, 90, 110, 180, 105, 180])   # Maximum v radiánech
limits_min = np.deg2rad([-175, -90, -110, -180, -105, -180])  # Minimum v radiánech

robot = ikt6_robot_init(
    name="CRS93",
    lengths=lengths,
    offsets=offsets,
    directions=directions,
    limits_max=limits_max,  # Horní meze pro každý kloub
    limits_min=limits_min,  # Dolní meze pro každý kloub
    base=base,
    tool=tool
)
```

**Uloženo v objektu Robot:**
```python
robot.limits_max = [3.054, 1.571, 1.920, 3.142, 1.833, 3.142]  # radiány
robot.limits_min = [-3.054, -1.571, -1.920, -3.142, -1.833, -3.142]
```

---

## 🔍 2. Funkce `test_limits()` - Kontrola Jednotlivých Kloubů

### Kde: `ikt6_python/kinematics.py` (řádek 50-63)

```python
def test_limits(robot: Robot, j: float, i: int) -> bool:
    """
    Test if a joint angle is within limits
    
    Args:
        robot: Robot parameters
        j: Joint angle (with offset and direction applied)
        i: Joint index (0-5)
    
    Returns:
        True if within limits
    """
    # Převede úhel zpět (odstraní offset a direction)
    j = j * robot.directions[i] + robot.offsets[i]
    
    # Kontrola: limits_min < j < limits_max
    return robot.limits_min[i] < j < robot.limits_max[i]
```

### Jak to funguje:

1. **Vstup `j`**: Úhel kloubu s aplikovaným offsetem a směrem
2. **Převod zpět**: `j = j * direction + offset` (vrátí originální úhel)
3. **Porovnání**: Zkontroluje, jestli je úhel mezi `limits_min[i]` a `limits_max[i]`
4. **Výstup**: `True` = v limitech, `False` = mimo limity

### Proč se převádí zpět?

- IK výpočty pracují s **transformovanými** úhly (s aplikovaným offsetem/směrem)
- Limity jsou definovány v **originálních** úhlech robotu
- Proto je potřeba převést zpět před kontrolou

---

## 🧮 3. Použití v IK Algoritmu (`ikt6_ikt`)

### A) Kontrola prvních 3 kloubů (θ1, θ2, θ3):

**Kde:** `kinematics.py`, řádky 238-241

```python
# Check limits
if (test_limits(robot, theta1, 0) and
    test_limits(robot, theta2, 1) and
    test_limits(robot, theta3, 2)):
    J3[:, i] = [theta1, theta2, theta3]  # Uloží pouze pokud jsou v limitech
```

**Co se děje:**
- Vypočítá `theta1`, `theta2`, `theta3` ze geometrie
- Zkontroluje každý úhel pomocí `test_limits()`
- **Pouze** pokud jsou všechny 3 v limitech → uloží jako platné řešení
- Pokud je **kterýkoli** mimo limity → zahodí řešení (zůstane `NaN`)

### B) Kontrola kloubu 6 při singularitě:

**Kde:** `kinematics.py`, řádky 264

```python
if c5 > (1 - EPS):  # Singularita: θ5 ≈ 0
    theta4 = 0
    theta5 = 0
    theta6 = atan2(-mtx[0,1], mtx[1,1]) - π
    theta6 = pmp(theta6)  # Normalizace na (-π, π]
    
    if test_limits(robot, theta6, 5):  # Kontrola pouze θ6
        J[:, n_sol_J] = [J3, theta4, theta5, theta6]
```

**V singularitě:**
- θ4 a θ5 jsou fixní (0)
- Zkontroluje pouze θ6

### C) Kontrola kloubů 4, 5, 6 (normální případ):

**Kde:** `kinematics.py`, řádky 288-291 a 299-302

```python
# Řešení A
if (test_limits(robot, theta4a, 3) and
    test_limits(robot, theta5a, 4) and
    test_limits(robot, theta6a, 5)):
    J[:, n_sol_J] = [J3, theta4a, theta5a, theta6a]

# Řešení B
if (test_limits(robot, theta4b, 3) and
    test_limits(robot, theta5b, 4) and
    test_limits(robot, theta6b, 5)):
    J[:, n_sol_J] = [J3, theta4b, theta5b, theta6b]
```

**Každé řešení:**
- Zkontroluje všechny 3 klouby (θ4, θ5, θ6)
- Uloží **pouze** pokud jsou **všechny** v limitech
- Každé řešení se kontroluje samostatně

---

## ✅ 4. Funkce `check_limits()` - Kontrola Celého Vektoru

### Kde: `ikt6_python/tests/test_utils.py` (řádek 34-45)

```python
def check_limits(robot: Robot, J: np.ndarray) -> bool:
    """
    Check if joint angles are within robot limits
    
    Args:
        robot: Robot parameters
        J: Joint angles (6 elements)
    
    Returns:
        True if all joints are within limits
    """
    return np.all(J > robot.limits_min) and np.all(J < robot.limits_max)
```

### Použití v testech:

```python
# Generuj náhodné úhly
J_in = np.zeros(6)
for j in range(6):
    J_in[j] = ((np.random.rand() - 0.5) * 
              (robot.limits_max[j] - robot.limits_min[j]))

# Kontrola před použitím
if not check_limits(robot, J_in):
    continue  # Přeskoč, pokud je mimo limity
```

---

## 🔄 5. Srovnání: `test_limits()` vs `check_limits()`

| Funkce | Použití | Vstup | Kontrola |
|--------|---------|-------|----------|
| **`test_limits()`** | V IK algoritmu | Jeden úhel + index | S odstraněním offset/direction |
| **`check_limits()`** | V testech | Celý vektor (6 úhlů) | Přímé porovnání |

---

## 📊 6. Příklad: Jak IK Filtruje Řešení

### Vstup:
- Cílová poze: `P = [500, 200, 600, 0, π/4, 0]`

### IK proces:

```
1. Vypočítá θ1a, θ1b (2 možnosti)
   ├─ θ1a = 0.380 rad  ✓ V limitech
   └─ θ1b = 3.521 rad  ✗ MIMO limity → zahodí

2. Pro θ1a vypočítá θ2, θ3 (2 možnosti)
   ├─ θ2a=-0.5, θ3a=-1.2  ✓ Obě v limitech
   └─ θ2b=1.8, θ3b=0.8    ✗ θ2b mimo → zahodí

3. Pro platné (θ1,θ2,θ3) vypočítá θ4,θ5,θ6 (2 možnosti)
   ├─ θ4a=0.1, θ5a=0.8, θ6a=-0.3  ✓ Všechny OK
   └─ θ4b=-0.2, θ5b=-0.8, θ6b=3.5  ✗ θ6b mimo → zahodí

Výsledek: 1 platné řešení z původních 8 možností
```

---

## 🛡️ 7. Co se stane s neplatnými řešeními?

### V IK výstupu:

```python
J_solutions = ikt6_ikt(robot, P=P_target)
# Shape: (6, 8) - až 8 řešení

# Neplatná řešení obsahují NaN:
# [[ 0.38, -0.50, -1.20,  0.10,  0.80, -0.30],  ← Platné
#  [  nan,   nan,   nan,   nan,   nan,   nan],  ← Mimo limity
#  [  nan,   nan,   nan,   nan,   nan,   nan],  ← Mimo limity
#  ...
# ]
```

### Detekce platných řešení:

```python
for i in range(J_solutions.shape[1]):
    if not np.any(np.isnan(J_solutions[:, i])):
        print(f"Platné řešení {i+1}: {J_solutions[:, i]}")
```

---

## 🎓 8. Shrnutí - Celý Proces

```
┌─────────────────────────────────────┐
│ 1. Definice limitů při inicializaci│
│    robot.limits_min, limits_max     │
└──────────┬──────────────────────────┘
           │
           ▼
┌─────────────────────────────────────┐
│ 2. IK výpočet (ikt6_ikt)            │
│    - Vypočítá θ1, θ2, θ3            │
│    - Zavolá test_limits() pro každý │
│    - Pouze platné → pokračuje       │
└──────────┬──────────────────────────┘
           │
           ▼
┌─────────────────────────────────────┐
│ 3. Výpočet θ4, θ5, θ6               │
│    - Pro každé platné řešení        │
│    - Zavolá test_limits() pro θ4-6  │
│    - Pouze platné → uloží           │
└──────────┬──────────────────────────┘
           │
           ▼
┌─────────────────────────────────────┐
│ 4. Výstup                           │
│    - 6×8 matice                     │
│    - Platná řešení: čísla           │
│    - Neplatná: NaN                  │
└─────────────────────────────────────┘
```

---

## 💡 9. Praktický Příklad

```python
import numpy as np
from ikt6_python import ikt6_robot_init, ikt6_ikt

# Robot s úzkými limity
limits_max = np.deg2rad([45, 45, 45, 180, 105, 180])  # Omezené
limits_min = np.deg2rad([-45, -45, -45, -180, -105, -180])

robot = ikt6_robot_init(
    name="CRS93_limited",
    lengths=np.array([440, 0, 305, 0, 330, 211.0]),
    offsets=np.zeros(6),
    directions=np.array([1, -1, -1, 1, -1, 1.0]),
    limits_max=limits_max,
    limits_min=limits_min
)

# IK pro vzdálenou pozici
P = np.array([800, 300, 500, 0, 0, 0])
J_sols = ikt6_ikt(robot, P=P)

# Počet platných řešení (pravděpodobně 0, protože je to daleko)
n_valid = sum(1 for i in range(8) if not np.any(np.isnan(J_sols[:, i])))
print(f"Platných řešení: {n_valid}/8")
# Výstup: "Platných řešení: 0/8" - pozice je mimo dosah
```

---

## ✨ Klíčové Body

1. ✅ **Automatická filtrace**: IK automaticky zahazuje řešení mimo limity
2. ✅ **Bezpečnost**: Nikdy nevrátí nebezpečnou konfiguraci
3. ✅ **Offsets/Directions**: Správně se převádí před kontrolou limitů
4. ✅ **Jednotlivá kontrola**: Každý kloub i každé řešení se kontroluje samostatně
5. ✅ **NaN značení**: Neplatná řešení jsou jasně označena jako `NaN`

---

**Závěr:** Knihovna IKT6 Python má vestavěnou robustní kontrolu limitů, která zajišťuje, že **všechna vrácená řešení jsou bezpečná a v limitech robotu**.
