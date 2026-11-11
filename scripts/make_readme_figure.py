#!/usr/bin/env python3
"""Generate the README figures from the stored EuRoC MH_01 results.

    python scripts/make_readme_figure.py

Reads results/MH_01/vins_result_no_loop.csv, summary.txt, evaluation/metrics_euroc.csv and
results/MH_01_groundtruth_tum.txt, aligns the trajectory to the groundtruth with SE(3) Umeyama
(as scripts/evaluate_euroc.py) and writes light and dark variants of two figures to docs/images/:
  * mh01_trajectory_{light,dark}.png - top view + position error over time
  * mh01_metrics_{light,dark}.png    - ATE by alignment, RPE over distance, runtime
Requires numpy and matplotlib.
"""
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
OUT = ROOT / "docs" / "images"
COL = "#d62728"

THEMES = {
    "light": dict(bg="white", fg="#24292f", grid="#d0d7de", gt="#24292f"),
    "dark": dict(bg="#0d1117", fg="#e6edf3", grid="#30363d", gt="#e6edf3"),
}


def load_vins(path):
    """vins_result_no_loop.csv: t[ns], px, py, pz, qw, qx, qy, qz, vx, vy, vz (original VINS-Mono format)."""
    rows = []
    with open(path, newline="") as f:
        for r in csv.reader(f):
            r = [x for x in r if x.strip()]
            if len(r) >= 4:
                try:
                    rows.append([float(r[0]) * 1e-9, float(r[1]), float(r[2]), float(r[3])])
                except ValueError:
                    pass
    return np.array(rows)


def align_se3(est, gt):
    t = est[:, 0]
    ok = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    t, pe = t[ok], est[ok, 1:4]
    pg = np.column_stack([np.interp(t, gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
    me, mg = pe.mean(0), pg.mean(0)
    U, _, Vt = np.linalg.svd((pg - mg).T @ (pe - me) / len(pe))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    pa = (R @ pe.T).T + (mg - R @ me)
    return t, pa, np.linalg.norm(pa - pg, axis=1)


def style(ax, th):
    ax.set_facecolor(th["bg"])
    for s in ax.spines.values():
        s.set_color(th["grid"])
    ax.tick_params(colors=th["fg"])
    for lab in (ax.xaxis.label, ax.yaxis.label, ax.title):
        lab.set_color(th["fg"])
    ax.grid(True, color=th["grid"], lw=0.6, alpha=0.8)
    ax.set_axisbelow(True)


def legend(ax, th, **kw):
    leg = ax.legend(fontsize=8.5, facecolor=th["bg"], edgecolor=th["grid"], **kw)
    for t in leg.get_texts():
        t.set_color(th["fg"])


def trajectory_figure(theme):
    th = THEMES[theme]
    gt = np.loadtxt(RES / "MH_01_groundtruth_tum.txt", comments="#")[:, :4]
    t, pa, err = align_se3(load_vins(RES / "MH_01" / "vins_result_no_loop.csv"), gt)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw=dict(width_ratios=[1, 1.35]))
    fig.patch.set_facecolor(th["bg"])
    a1.plot(gt[:, 1], gt[:, 2], color=th["gt"], lw=2.2, ls="--", label="Groundtruth (Leica + Vicon)")
    a1.plot(pa[:, 0], pa[:, 1], color=COL, lw=1.1, label="VINS-Mono (no loop)")
    a2.plot(t - gt[0, 0], err, color=COL, lw=1.0, label=f"VINS-Mono  (ATE RMSE {np.sqrt(np.mean(err**2)):.3f} m)")
    a1.set_aspect("equal", adjustable="datalim")
    a1.set_xlabel("x [m]"); a1.set_ylabel("y [m]")
    a1.set_title("EuRoC MH_01_easy - top view (SE(3) aligned)")
    a2.set_xlabel("time [s]"); a2.set_ylabel("position error [m]")
    a2.set_title("Absolute position error over time")
    a2.set_ylim(0, None)
    for ax in (a1, a2):
        style(ax, th)
        legend(ax, th, loc="best")
    fig.tight_layout()
    fig.savefig(OUT / f"mh01_trajectory_{theme}.png", dpi=150, facecolor=th["bg"])
    plt.close(fig)


def metrics_figure(theme):
    th = THEMES[theme]
    with open(RES / "MH_01" / "evaluation" / "metrics_euroc.csv", newline="") as f:
        m = {k: float(v) for k, v in next(csv.DictReader(f)).items() if k != "run"}
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.2), gridspec_kw=dict(width_ratios=[1, 1.4, 0.8]))
    fig.patch.set_facecolor(th["bg"])

    a = axs[0]
    names = ["SE(3)", "4-DoF", "Sim(3)"]
    vals = [m["ate_SE3_rmse_m"], m["ate_4DoF_rmse_m"], m["ate_Sim3_rmse_m"]]
    bars = a.bar(names, vals, 0.6, color=["#1f77b4", "#8250df", "#2ca02c"])
    for b, v in zip(bars, vals):
        a.text(b.get_x() + b.get_width() / 2, v, f"{v:.3f}", ha="center", va="bottom", fontsize=9, color=th["fg"])
    a.set_ylabel("ATE RMSE [m]"); a.set_title("Absolute trajectory error by alignment")

    a = axs[1]
    d = [8, 16, 24, 32, 40]
    pct = [m[f"rpe{k}m_trans_pct"] for k in d]
    rot = [m[f"rpe{k}m_rot_deg"] for k in d]
    a.plot(d, pct, "o-", color=COL, label="translation [% of distance]")
    a.plot(d, rot, "s--", color="#bf8700", label="rotation [deg]")
    for x, p in zip(d, pct):
        a.text(x, p, f" {p:.2f}%", fontsize=8, color=th["fg"], va="bottom")
    a.set_xticks(d); a.set_xlabel("travelled distance [m]")
    a.set_title("Relative pose error (mean)")
    legend(a, th)

    a = axs[2]
    summ = dict(l.strip().split("=", 1) for l in open(RES / "MH_01" / "summary.txt") if "=" in l)
    solve, track = m["mean_solve_ms"], float(summ.get("mean_track_ms", "nan"))
    a.bar(["per frame"], [solve], 0.5, color=COL, label="sliding-window solver")
    if np.isfinite(track):
        a.bar(["per frame"], [track], 0.5, bottom=[solve], color="#57606a", label="feature tracker")
    total = solve + (track if np.isfinite(track) else 0)
    a.text(0, total, f"{total:.1f} ms\n{m['realtime_factor']:.1f}x real time", ha="center", va="bottom",
           fontsize=9, color=th["fg"])
    a.set_ylabel("mean time [ms]"); a.set_title("Runtime (i9-13950HX)")
    legend(a, th, loc="lower right")
    for ax in axs:
        style(ax, th)
    for ax in axs[:2]:
        ax.set_ylim(0, None)
        ax.margins(y=0.2)
    axs[2].set_ylim(0, total * 1.4)
    fig.tight_layout()
    fig.savefig(OUT / f"mh01_metrics_{theme}.png", dpi=150, facecolor=th["bg"])
    plt.close(fig)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for theme in THEMES:
        trajectory_figure(theme)
        metrics_figure(theme)
    print("figures written to", OUT)
