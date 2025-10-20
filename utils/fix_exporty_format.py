#!/usr/bin/env python3
"""
Skript pro opravu formátu všech data_*.yaml souborů v exporty/
Přidá čárky a správné formátování podle pos.yaml
"""

import re
from pathlib import Path
import numpy as np


def parse_array_from_text(text):
    """Parsuje numpy array z textového řetězce."""
    # Odstraníme hranatá zátvorky a rozdělíme na čísla
    text = text.strip()
    text = text.replace('[', '').replace(']', '').replace('\n', ' ')
    numbers = [float(x) for x in text.split() if x]
    return np.array(numbers)


def parse_matrix_from_text(text):
    """Parsuje 2D matici z textového řetězce."""
    text = text.strip()
    # Najdeme všechna čísla
    numbers = re.findall(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?', text)
    numbers = [float(x) for x in numbers]
    
    # Předpokládáme 4x4 transformační matici
    if len(numbers) == 16:
        matrix = np.array(numbers).reshape(4, 4)
        return matrix
    else:
        # Pokud to není 16 čísel, vraťme jako 1D
        arr = np.array(numbers)
        # Zkusíme odhadnout, jestli by to mohla být matice
        if len(numbers) > 0:
            sqrt = int(np.sqrt(len(numbers)))
            if sqrt * sqrt == len(numbers):
                return arr.reshape(sqrt, sqrt)
        return arr


def format_1d_array(arr):
    """Formátuje 1D pole s čárkami."""
    return "[" + ", ".join(f"{x: .8e}" for x in arr) + "]"


def format_2d_matrix(arr):
    """Formátuje 2D matici s čárkami a odsazením."""
    rows = ["[" + ", ".join(f"{x: .8e}" for x in row) + "]" for row in arr]
    return "[" + ",\n       ".join(rows) + "]"


def fix_yaml_file(filepath):
    """Opraví formát jednoho YAML souboru."""
    print(f"Opravuji: {filepath.name}")
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Parsujeme řádek po řádku
    robot_q = None
    matrix_rows = []
    in_matrix = False
    
    for line in lines:
        if 'Robot_pos_q' in line:
            # Extrahujeme čísla z řádku
            match = re.search(r'\[(.*?)\]', line)
            if match:
                robot_q = parse_array_from_text(match.group(1))
        elif 'transformacni_matic' in line:
            in_matrix = True
            # Pokud je matice na stejném řádku
            if '[' in line:
                rest = line.split('[', 1)[1]
                if ']' not in rest or rest.count('[') > 0:
                    continue  # Matice pokračuje na dalších řádcích
        elif in_matrix:
            # Hledáme řádky matice
            if '[' in line and ']' in line:
                # Celý řádek matice
                match = re.search(r'\[(.*?)\]', line)
                if match:
                    row_text = match.group(1)
                    row_numbers = [float(x) for x in row_text.split() if x]
                    if row_numbers:
                        matrix_rows.append(row_numbers)
            if line.strip() == ']':
                in_matrix = False
    
    if robot_q is None:
        print(f"  ⚠️  Nepodařilo se najít Robot_pos_q")
        return False
    
    if not matrix_rows or len(matrix_rows) != 4:
        print(f"  ⚠️  Nepodařilo se načíst matici 4x4 (našlo se {len(matrix_rows)} řádků)")
        return False
    
    # Vytvoříme numpy matici
    matrix = np.array(matrix_rows)
    
    # Formátujeme
    robot_q_formatted = format_1d_array(robot_q)
    matrix_formatted = format_2d_matrix(matrix)
    
    # Vytvoříme nový obsah
    new_content = f"Robot_pos_q : {robot_q_formatted}\n\n"
    new_content += f"transformacni_matic : {matrix_formatted}\n"
    
    # Uložíme zpět
    with open(filepath, 'w') as f:
        f.write(new_content)
    
    print(f"  ✅ Opraveno")
    return True


def main():
    # Najdeme všechny data_*.yaml soubory
    exporty_dir = Path(__file__).parent / "exporty"
    
    if not exporty_dir.exists():
        print(f"❌ Složka {exporty_dir} neexistuje!")
        return
    
    yaml_files = sorted(exporty_dir.glob("data_*.yaml"))
    
    if not yaml_files:
        print(f"❌ Nenalezeny žádné soubory data_*.yaml v {exporty_dir}")
        return
    
    print(f"Nalezeno {len(yaml_files)} souborů k opravě\n")
    print("=" * 60)
    
    success_count = 0
    for filepath in yaml_files:
        if fix_yaml_file(filepath):
            success_count += 1
        print()
    
    print("=" * 60)
    print(f"Hotovo! Úspěšně opraveno {success_count}/{len(yaml_files)} souborů")


if __name__ == "__main__":
    main()
