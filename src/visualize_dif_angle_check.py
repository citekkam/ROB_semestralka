#!/usr/bin/env python3
"""
Enhanced visualization of joint angle differences:
- Shows raw (absolute) joint deltas between current and target
- Shows normalized (wrapped to [-pi, pi]) deltas for comparison
- Highlights joints that exceed provided per-joint limits
- Demonstrates effect of wrap-around (e.g. +170 -> -170)
- Optional sequence mode: interpolate between current and target and plot evolution

Usage examples:
  python src/visualize_dif_angle_check.py --current 170 0 0 0 0 0 --target -170 0 0 0 0 0 --show-normalized
  python src/visualize_dif_angle_check.py --current 0 0 0 0 0 0 --target 0 0 0 10 170 0 --show-normalized
  python src/visualize_dif_angle_check.py --current 0 80 0 0 0 0 --target 0 95 0 0 0 0 --limits 175 90 110 180 105 180
  python src/visualize_dif_angle_check.py --sequence --steps 40 --current 170 0 0 0 0 0 --target -170 0 0 0 0 0

"""
import argparse
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_LIMITS_DEG = np.array([175.0, 90.0, 110.0, 180.0, 105.0, 180.0])

def normalize_angle(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))

def raw_diffs_deg(qc: np.ndarray, qt: np.ndarray) -> np.ndarray:
    return np.rad2deg(np.abs(qt - qc))

def norm_diffs_deg(qc: np.ndarray, qt: np.ndarray) -> np.ndarray:
    delta = qt - qc
    return np.rad2deg(np.abs(np.array([normalize_angle(d) for d in delta])))

def check_limits(diff_deg: np.ndarray, limits_deg: np.ndarray) -> np.ndarray:
    return diff_deg <= limits_deg

def wrist_flip(qc: np.ndarray, qt: np.ndarray, thresh_deg: float = 150.0) -> bool:
    d = np.rad2deg(np.abs(qt - qc))[3:]
    return np.any(d > thresh_deg)

def interpolate_joint_space(qc: np.ndarray, qt: np.ndarray, steps: int) -> np.ndarray:
    # Linear interpolation in joint space (not necessarily Cartesian linear)
    alphas = np.linspace(0, 1, steps)
    return np.array([qc + a * (qt - qc) for a in alphas])

def plot_single(raw_deg, norm_deg, limits_deg, raw_pass, norm_pass, flip_flag, joints_labels):
    fig, axs = plt.subplots(1, 2, figsize=(12,4), constrained_layout=True)
    for ax, values, passed, title in [
        (axs[0], raw_deg, raw_pass, 'Raw diffs (abs target - current)'),
        (axs[1], norm_deg, norm_pass, 'Normalized diffs (wrapped)')
    ]:
        colors = ['tab:green' if p else 'tab:red' for p in passed]
        ax.bar(joints_labels, values, color=colors, alpha=0.85)
        ax.plot(joints_labels, limits_deg, 'k--', label='limit')
        ax.set_ylabel('Degrees')
        ax.set_xlabel('Joint')
        ax.set_title(title)
        ax.grid(True, linestyle=':', alpha=0.5)
        ax.legend()
    fig.suptitle(f'Joint difference comparison (wrist flip: {"YES" if flip_flag else "NO"})')
    return fig

def plot_sequence(q_seq: np.ndarray, qc: np.ndarray, qt: np.ndarray, limits_deg: np.ndarray):
    steps = q_seq.shape[0]
    raw_list = []
    norm_list = []
    for q in q_seq:
        raw_list.append(raw_diffs_deg(qc, q))
        norm_list.append(norm_diffs_deg(qc, q))
    raw_arr = np.array(raw_list)
    norm_arr = np.array(norm_list)

    fig, axs = plt.subplots(2, 1, figsize=(10,8), constrained_layout=True)
    for arr, ax, title in [(raw_arr, axs[0], 'Raw diffs vs step'), (norm_arr, axs[1], 'Normalized diffs vs step')]:
        for j in range(arr.shape[1]):
            ax.plot(arr[:, j], label=f'J{j+1}')
        ax.set_xlabel('Step')
        ax.set_ylabel('Degrees')
        ax.set_title(title)
        ax.grid(True, linestyle=':', alpha=0.4)
        # Plot limits as horizontal lines (raw only logically; keep for reference)
        for j, lim in enumerate(limits_deg):
            ax.axhline(lim, color='k', linestyle='--', linewidth=0.6)
        ax.legend(ncol=3)

    fig.suptitle('Evolution of joint differences during interpolation')
    return fig

def main():
    parser = argparse.ArgumentParser(description='Visualize joint angle differences (raw vs normalized).')
    parser.add_argument('--current', nargs=6, type=float, default=[0,0,0,0,0,0], metavar=('J1','J2','J3','J4','J5','J6'))
    parser.add_argument('--target', nargs=6, type=float, default=[10,5,-15,20,15,-10], metavar=('J1','J2','J3','J4','J5','J6'))
    parser.add_argument('--limits', nargs=6, type=float, default=DEFAULT_LIMITS_DEG.tolist())
    parser.add_argument('--show-normalized', action='store_true', help='Display normalized comparison (enabled by default in dual plot).')
    parser.add_argument('--orient-thresh', type=float, default=150.0, help='Threshold for wrist flip heuristic (deg).')
    parser.add_argument('--sequence', action='store_true', help='Also show evolution along interpolation between current and target.')
    parser.add_argument('--steps', type=int, default=40, help='Interpolation steps for sequence mode.')

    args = parser.parse_args()

    qc_deg = np.array(args.current, dtype=float)
    qt_deg = np.array(args.target, dtype=float)
    limits_deg = np.array(args.limits, dtype=float)
    qc = np.deg2rad(qc_deg)
    qt = np.deg2rad(qt_deg)

    raw_deg = raw_diffs_deg(qc, qt)
    norm_deg = norm_diffs_deg(qc, qt)
    raw_pass = check_limits(raw_deg, limits_deg)
    norm_pass = check_limits(norm_deg, limits_deg)
    flip_flag = wrist_flip(qc, qt, thresh_deg=args.orient_thresh)

    print('Current (deg):', qc_deg)
    print('Target  (deg):', qt_deg)
    print('Per-joint limits (deg):', limits_deg)
    print('RAW diffs (deg):', raw_deg, 'pass mask:', raw_pass)
    print('NORM diffs (deg):', norm_deg, 'pass mask:', norm_pass)
    print('Wrist flip heuristic:', 'YES' if flip_flag else 'NO')

    joints_labels = [f'J{i+1}' for i in range(6)]
    fig_main = plot_single(raw_deg, norm_deg, limits_deg, raw_pass, norm_pass, flip_flag, joints_labels)

    figs = [fig_main]
    if args.sequence:
        q_seq = interpolate_joint_space(qc, qt, args.steps)
        fig_seq = plot_sequence(q_seq, qc, qt, limits_deg)
        figs.append(fig_seq)

    plt.show()

if __name__ == '__main__':
    main()
