import numpy as np
import cv2
import os
import yaml


def load_translations(folder="exporty", prefix="data_", start=1, end=30):
    rows = []
    for i in range(start, end + 1):
        path = os.path.join(folder, f"{prefix}{i}.yaml")
        if not os.path.exists(path):
            print(f"Chybí soubor: {path}")
            continue

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        T = np.array(data["transformacni_matic"], dtype=float)  # očekává se 4x4
        rows.append([T[0, 3], T[1, 3], T[2, 3]])  # prvky 1,4; 2,4; 3,4

    return np.asarray(rows, dtype=float)  # tvar (N, 3)

def find_global_homography_from_circles(translations_xyz: np.ndarray,
                                        folder="exporty", prefix="data_", ext=".png",
                                        ransac_thresh_m=0.003):
    """
    translations_xyz: np.ndarray shape (N,3) s [tx, ty, tz] pro každý obrázek
    Vrací: H (3x3, image->world), mask (inliers), img_pts (Nx2), world_xy (Nx2)
    """
    img_pts = []
    world_xy = []

    N = translations_xyz.shape[0]
    for i in range(1, N+1):
        path = os.path.join(folder, f"{prefix}{i}{ext}")
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            print(f"[WARN] Chybí obrázek: {path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,
                                   dp=1.2, minDist=120,
                                   param1=110, param2=32,
                                   minRadius=40, maxRadius=140)

        if circles is None:
            print(f"[WARN] Kruh v {path} nenalezen.")
            continue

        # použij plovoucí čárku; vyber nejlepší kandidát jednoduchým skóre
        C = circles[0] if circles.ndim == 3 else circles
        Hh, Ww = gray.shape[:2]
        scores = [float(r) + 0.002*(Ww - x) + 0.002*y for (x, y, r) in C]
        x, y, r = C[int(np.argmax(scores))]  # float

        # odpovídající (X,Y) z translace
        X, Y = translations_xyz[i-1, :2]

        img_pts.append([x, y])
        world_xy.append([X, Y])

    img_pts = np.asarray(img_pts, dtype=float).reshape(-1, 1, 2)
    world_xy = np.asarray(world_xy, dtype=float).reshape(-1, 1, 2)

    if len(img_pts) < 4:
        raise RuntimeError(f"Málo korespondencí: {len(img_pts)}. Potřebujeme aspoň 4 (z různých poloh na stejné rovině).")

    H, mask = cv2.findHomography(img_pts, world_xy,
                                 method=cv2.RANSAC,
                                 ransacReprojThreshold=ransac_thresh_m)
    return H, mask, img_pts.reshape(-1,2), world_xy.reshape(-1,2)

# volitelně: rychlá kontrola kvality
def homography_report(H, img_pts, world_xy):
    def to_h(P): return np.hstack([P, np.ones((P.shape[0],1))])
    # image -> world
    Pw_h = (H @ to_h(img_pts).T).T
    Pw = Pw_h[:, :2] / Pw_h[:, 2:3]
    err_w = np.linalg.norm(Pw - world_xy, axis=1)
    rmse_w = float(np.sqrt(np.mean(err_w**2)))
    # world -> image
    Hinv = np.linalg.inv(H)
    Pi_h = (Hinv @ to_h(world_xy).T).T
    Pi = Pi_h[:, :2] / Pi_h[:, 2:3]
    err_i = np.linalg.norm(Pi - img_pts, axis=1)
    rmse_i = float(np.sqrt(np.mean(err_i**2)))
    print(f"RMSE image->world: {rmse_w:.4f} m | world->image: {rmse_i:.2f} px | inliers: {len(err_w)}")

if __name__ == "__main__":
    translations = load_translations("exporty", "data_", 1, 30)   # (N,3)

    H, mask, img_pts, world_xy = find_global_homography_from_circles(
        translations, folder="exporty", prefix="data_", ext=".png",
        ransac_thresh_m=0.003
    )
    print("H (image->world):\n", H)
    homography_report(H, img_pts, world_xy)

    # Příklad použití: převod nového detekovaného pixelu (u,v) na světové XY
    # u,v = 512.3, 401.8
    # p = np.array([u, v, 1.0])
    # pw = H @ p
    # X, Y = (pw[0]/pw[2], pw[1]/pw[2])
