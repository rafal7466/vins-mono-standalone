#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
evaluate_euroc.py -- accuracy of the VINS-Mono C++ port on a EuRoC sequence against the Leica/Vicon ground truth
(state_groundtruth_estimate0/data.csv: pose of the IMU body frame, velocity, gyro / accel biases).

Metrics (as in the VINS-Mono paper / evo / Zhang & Scaramuzza IROS 2018):
  * ATE (position RMSE, mean, median, max) after SE(3) alignment (rotation + translation, evo "-a"),
    after 4-DoF alignment (yaw + translation: the unobservable directions of a VIO) and after Sim(3) (scale diagnostic);
  * RPE: relative translation / rotation error over 8, 16, 24, 32, 40 m of travelled distance;
  * velocity error, estimated vs ground-truth IMU biases, initialisation time, reboots, run time.
usage:
  python tools/evaluate_euroc.py --run output/MH01 [--run output/MH01_run2 ...] --gt <MH_01>/state_groundtruth_estimate0/data.csv --out output/evaluation
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def quat_to_R(q):  # q = [w, x, y, z] (N,4)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def umeyama(src, dst, scale=False):
    ms, md = src.mean(0), dst.mean(0)
    a, b = src - ms, dst - md
    C = b.T @ a / len(src)
    U, S, Vt = np.linalg.svd(C)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (a ** 2).sum(1).mean() if scale else 1.0
    return s, R, md - s * R @ ms


def align_4dof(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    a, b = src[:, :2] - ms[:2], dst[:, :2] - md[:2]
    yaw = np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    return 1.0, R, md - R @ ms


def stats(e):
    return dict(rmse=float(np.sqrt(np.mean(e ** 2))), mean=float(np.mean(e)), median=float(np.median(e)), max=float(np.max(e)), std=float(np.std(e)))


def rpe(P, Pg, R, Rg, dists):
    seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(Pg, axis=0), axis=1))]
    out = {}
    for L in dists:
        et, er = [], []
        for i in range(0, len(P), 5):
            j = np.searchsorted(seg, seg[i] + L)
            if j >= len(P):
                break
            # relative motion in the frame of pose i (alignment-free)
            dE = R[i].T @ (P[j] - P[i]); dG = Rg[i].T @ (Pg[j] - Pg[i])
            et.append(np.linalg.norm(dE - dG))
            Rerr = (Rg[i].T @ Rg[j]).T @ (R[i].T @ R[j])
            er.append(np.degrees(np.arccos(np.clip((np.trace(Rerr) - 1) / 2, -1, 1))))
        if et:
            out[L] = dict(n=len(et), trans_mean=float(np.mean(et)), trans_median=float(np.median(et)), trans_pct=float(100 * np.mean(et) / L),
                          rot_mean_deg=float(np.mean(er)), rot_deg_per_m=float(np.mean(er) / L))
    return out, seg[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--out", default="output/evaluation")
    ap.add_argument("--rpe", default="8,16,24,32,40")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    G = pd.read_csv(a.gt)
    G.columns = [c.strip() for c in G.columns]
    tg = G.iloc[:, 0].values.astype(np.int64)
    PG = G.iloc[:, 1:4].values; QG = G.iloc[:, 4:8].values; VG = G.iloc[:, 8:11].values; BGg = G.iloc[:, 11:14].values; BAg = G.iloc[:, 14:17].values
    dists = [float(x) for x in a.rpe.split(",")]
    rows, runs = [], []
    for rd in a.run:
        V = pd.read_csv(os.path.join(rd, "vins_trajectory.csv"))
        summ = {}
        sf = os.path.join(rd, "summary.txt")
        if os.path.isfile(sf):
            for line in open(sf):
                if "=" in line:
                    k, v = line.strip().split("=", 1); summ[k] = v
        t = V.timestamp_ns.values.astype(np.int64)
        m = (t >= tg[0]) & (t <= tg[-1])
        V = V[m].reset_index(drop=True); t = t[m]
        # ground truth at the estimate times (linear for position / velocity / biases, nearest for attitude)
        tf, tgf = (t - tg[0]) * 1e-9, (tg - tg[0]) * 1e-9
        Pg = np.column_stack([np.interp(tf, tgf, PG[:, k]) for k in range(3)])
        Vg = np.column_stack([np.interp(tf, tgf, VG[:, k]) for k in range(3)])
        bg_g = np.column_stack([np.interp(tf, tgf, BGg[:, k]) for k in range(3)])
        ba_g = np.column_stack([np.interp(tf, tgf, BAg[:, k]) for k in range(3)])
        idx = np.clip(np.searchsorted(tg, t), 0, len(tg) - 1)
        Rg = quat_to_R(QG[idx])
        P = V[["px", "py", "pz"]].values; R = quat_to_R(V[["qw", "qx", "qy", "qz"]].values); Vel = V[["vx", "vy", "vz"]].values
        res = {}
        for name, fn in [("SE3", lambda s, d: umeyama(s, d, False)), ("4DoF", align_4dof), ("Sim3", lambda s, d: umeyama(s, d, True))]:
            s, Ra, ta = fn(P, Pg)
            Pa = (s * (Ra @ P.T)).T + ta
            e = np.linalg.norm(Pa - Pg, axis=1)
            res[name] = dict(scale=s, R=Ra, t=ta, P=Pa, err=e, stats=stats(e))
        Ra = res["SE3"]["R"]
        Va = (Ra @ Vel.T).T
        ev = np.linalg.norm(Va - Vg, axis=1)
        rp, length = rpe(P, Pg, R, Rg, dists)
        lab = os.path.basename(os.path.normpath(rd))
        row = dict(run=lab, poses=len(P), path_m=float(length), duration_s=float(tf[-1] - tf[0]), init_s=float(summ.get("first_init_t_rel_s", "nan")),
                   reboots=int(float(summ.get("failures_reboots", "0"))), realtime_factor=float(summ.get("realtime_factor", "nan")),
                   mean_solve_ms=float(summ.get("mean_solve_ms", "nan")))
        for n in ["SE3", "4DoF", "Sim3"]:
            for k, v in res[n]["stats"].items():
                row[f"ate_{n}_{k}_m"] = v
        row["sim3_scale"] = res["Sim3"]["scale"]
        row["ate_SE3_pct_of_path"] = 100 * res["SE3"]["stats"]["rmse"] / length
        row["vel_err_rms_mps"] = float(np.sqrt(np.mean(ev ** 2)))
        row["gyro_bias_err_rms_rad_s"] = float(np.sqrt(np.mean(np.sum((V[["bgx", "bgy", "bgz"]].values - bg_g) ** 2, 1))))
        row["acc_bias_err_rms_m_s2"] = float(np.sqrt(np.mean(np.sum((V[["bax", "bay", "baz"]].values - ba_g) ** 2, 1))))
        for L, r in rp.items():
            row[f"rpe{int(L)}m_trans_mean_m"] = r["trans_mean"]; row[f"rpe{int(L)}m_trans_pct"] = r["trans_pct"]; row[f"rpe{int(L)}m_rot_deg"] = r["rot_mean_deg"]
        rows.append(row)
        runs.append(dict(lab=lab, t=tf, P=P, Pg=Pg, res=res, Vel=Va, Vg=Vg, V=V, bg_g=bg_g, ba_g=ba_g, rp=rp))
    M = pd.DataFrame(rows)
    M.to_csv(os.path.join(a.out, "metrics_euroc.csv"), index=False, float_format="%.5f")

    # ---------------- figures (first run) ----------------
    r = runs[0]
    B, O, GR, TX = "#2a78d6", "#eb6834", "#8a8984", "#52514e"
    plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False, "font.size": 10})
    Pa = r["res"]["SE3"]["P"]
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.2))
    ax[0].plot(r["Pg"][:, 0], r["Pg"][:, 1], color=GR, lw=3, label="ground truth")
    ax[0].plot(Pa[:, 0], Pa[:, 1], color=B, lw=1.4, label="VINS-Mono C++ (SE3-aligned)")
    ax[0].plot(Pa[0, 0], Pa[0, 1], "s", color="k", ms=6); ax[0].axis("equal"); ax[0].grid(alpha=.3)
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]"); ax[0].legend(frameon=False); ax[0].set_title("Top view")
    ax[1].plot(r["t"], r["res"]["SE3"]["err"], color=B, lw=1.2, label="SE3 alignment")
    ax[1].plot(r["t"], r["res"]["4DoF"]["err"], color=O, lw=1.0, label="4-DoF alignment")
    ax[1].set_xlabel("time from first pose [s]"); ax[1].set_ylabel("position error [m]"); ax[1].grid(alpha=.3); ax[1].legend(frameon=False)
    ax[1].set_title(f"ATE  (SE3 RMSE {r['res']['SE3']['stats']['rmse']:.3f} m)")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_trajectory_ate.png"), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    for k, c in enumerate("xyz"):
        ax[k, 0].plot(r["t"], r["Pg"][:, k], color=GR, lw=2.5, label="ground truth"); ax[k, 0].plot(r["t"], Pa[:, k], color=B, lw=1, label="VINS-Mono")
        ax[k, 0].set_ylabel(f"p_{c} [m]"); ax[k, 0].grid(alpha=.3)
        ax[k, 1].plot(r["t"], r["Vg"][:, k], color=GR, lw=2.5); ax[k, 1].plot(r["t"], r["Vel"][:, k], color=B, lw=1)
        ax[k, 1].set_ylabel(f"v_{c} [m/s]"); ax[k, 1].grid(alpha=.3)
    ax[0, 0].legend(frameon=False); ax[2, 0].set_xlabel("time [s]"); ax[2, 1].set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_position_velocity.png"), dpi=130); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(13, 4))
    V = r["V"]
    for k, (c, col) in enumerate(zip("xyz", [B, O, "#1baf7a"])):
        ax[0].plot(r["t"], r["ba_g"][:, k], color=col, lw=2.5, alpha=.35); ax[0].plot(r["t"], V[f"ba{c}"].values, color=col, lw=1, label=f"b_a{c}")
        ax[1].plot(r["t"], r["bg_g"][:, k], color=col, lw=2.5, alpha=.35); ax[1].plot(r["t"], V[f"bg{c}"].values, color=col, lw=1, label=f"b_g{c}")
    ax[0].set_title("accelerometer bias [m/s²] (thick: ground truth)"); ax[1].set_title("gyro bias [rad/s] (thick: ground truth)")
    for x in ax: x.grid(alpha=.3); x.legend(frameon=False, ncol=3); x.set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "fig_biases.png"), dpi=130); plt.close(fig)

    with open(os.path.join(a.out, "report.txt"), "w", encoding="utf-8") as f:
        f.write("VINS-Mono C++ port on EuRoC (" + a.gt + ")\n\n")
        f.write(M.T.to_string(float_format=lambda x: f"{x:.4f}") + "\n")
        if len(M) > 1:
            f.write("\nmean +- std over runs: ATE SE3 RMSE %.4f +- %.4f m, 4DoF %.4f +- %.4f m\n" % (M.ate_SE3_rmse_m.mean(), M.ate_SE3_rmse_m.std(), M["ate_4DoF_rmse_m"].mean(), M["ate_4DoF_rmse_m"].std()))
    print(open(os.path.join(a.out, "report.txt"), encoding="utf-8").read())
    return 0


if __name__ == "__main__":
    sys.exit(main())
