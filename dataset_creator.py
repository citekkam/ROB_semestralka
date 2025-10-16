import numpy as np
from PIL import Image
import os

# --- 1️⃣ Funkce pro automatické pojmenování souboru ---
def next_filename(prefix="data", ext=".npz"):
    i = 1
    while True:
        filename = f"{prefix}_{i}{ext}"
        if not os.path.exists(filename):
            return filename
        i += 1

# --- 2️⃣ Vytvoření matic ---
A = np.array([[1, 2, 3],
              [4, 5, 6]])
B = np.array([[7, 8],
              [9, 10]])

# --- 3️⃣ Načtení nebo vytvoření obrázku ---
# Pokud máš vlastní obrázek, odkomentuj tento řádek:
# img = np.array(Image.open("obrazek.jpg"))

# Jinak se vytvoří náhodný obrázek:
img = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)

# --- 4️⃣ Získání unikátního jména souboru ---
filename = next_filename(prefix="data", ext=".npz")

# --- 5️⃣ Uložení dat ---
np.savez(filename, obrazek=img, matice_A=A, matice_B=B)
print(f"✅ Obrázek a matice byly uloženy do souboru: {filename}")

# --- 6️⃣ Uložení obrázku i jako PNG se stejným číslem ---
png_name = filename.replace(".npz", ".png")
Image.fromarray(img).save(png_name)
print(f"✅ Obrázek uložen jako: {png_name}")
