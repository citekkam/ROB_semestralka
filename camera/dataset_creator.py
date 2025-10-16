import numpy as np
import cv2
import os
from pathlib import Path

# --- 1️⃣ Funkce pro automatické pojmenování ---
def next_filename(folder="exporty", prefix="data", ext=".yaml"):
    # Vytvoříme cestu relativně ke složce camera/
    base_path = Path(__file__).parent / folder
    base_path.mkdir(exist_ok=True)
    i = 1
    while True:
        filename = base_path / f"{prefix}_{i}{ext}"
        if not filename.exists():
            return str(filename)
        i += 1


# --- 2️⃣ Formátování matice do textu ---
def format_array(arr):
    """Vrátí pole jako hezky zformátovaný text podobný numpy výstupu."""
    if arr.ndim == 1:
        return "[" + " ".join(f"{x: .8e}" for x in arr) + "]"
    elif arr.ndim == 2:
        rows = ["[" + " ".join(f"{x: .8e}" for x in row) + "]" for row in arr]
        return "[\n " + "\n ".join(rows) + "\n]"
    else:
        return str(arr)


# --- 3️⃣ Hlavní funkce pro uložení ---
def uloz_data(robot_q, transformation_matrix, obrazek, folder="exporty", prefix="data"):
    """
    Uloží dvě matice do .yaml souboru v textovém formátu a obrázek jako .png.
    """
    filename_yaml = next_filename(folder=folder, prefix=prefix, ext=".yaml")
    filename_png = filename_yaml.replace(".yaml", ".png")

    # --- Vytvoření obsahu YAML ---
    text = ""
    text += "Robot_pos_q:  " + format_array(robot_q) + "\n"
    text += "transformacni_matic: " + format_array(transformation_matrix) + "\n"

    # --- Uložení do souboru ---
    with open(filename_yaml, "w") as f:
        f.write(text)
    print(f"✅ Matice uloženy do: {filename_yaml}")

    # --- Uložení obrázku ---
    cv2.imwrite(filename_png, obrazek)
    print(f"✅ Obrázek uložen jako: {filename_png}")

    return filename_yaml, filename_png


# --- 4️⃣ Příklad použití ---
if __name__ == "__main__":
    robot_q = np.array([
        1.25663706e-04, -1.15965610e+00, -1.54057421e+00,
        6.22097555e-05, -4.34796423e-01, -3.11048778e-05
    ])

    transformation_matrix = np.array([
        [-9.99978439e-01, -1.00519685e-04,  6.56587886e-03,  4.20952110e-01],
        [-1.00344376e-04,  9.99999995e-01,  2.70294453e-05,  5.48951742e-05],
        [-6.56588154e-03,  2.63700136e-05, -9.99978444e-01,  5.22593112e-02],
        [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
    ])

    obrazek = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)

    uloz_data(robot_q, transformation_matrix, obrazek)
