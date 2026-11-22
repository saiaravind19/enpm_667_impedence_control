"""
Stability region in the impedance-parameter space (paper Sec. V, Figs. 2-3).

Procedure (simulated version of the paper's experiment): the robot is held at the
start pose q_i with an isotropic desired impedance  Lambda_d = alpha * lambda_bar * I,
D_d = D * I  (lambda_bar = largest eigenvalue of Lambda(q_i)). A "human" grasps the pen
with a stiff, time-varying grip (stiffness varying between k_lo and k_hi) and shakes
it around the start point. For each (alpha, D) we check whether the motion stays smooth
or develops a growing high-frequency vibration.

What makes the loop able to go unstable here (as on the real robot): the external force
is not measured but *estimated* by the first-order momentum observer (bandwidth K_I),
and the torque command reaches the joints with a small latency (default 2 ms, typical
of the LWR4 FRI interface). When lambda_d << Lambda, the force-feedback gain
(Lambda Lambda_d^-1 - I) in eq. (5) becomes large and this lag destabilizes the
contact with the stiff human hand.

    python stability.py            # sweep, saves results/stability_region.png + .npz
"""

from __future__ import annotations

import argparse
import os
from collections import deque

import mujoco
import numpy as np

from vic import Q_INIT, load_model, Arm, ImpedanceLaw, VariableImpedanceController

HERE = os.path.dirname(os.path.abspath(__file__))


def shake_test(alpha, D, T=3.0, delay_steps=2, K_I=100.0, k_lo=200.0, k_hi=2000.0,
               amp=0.01, f_shake=2.0, redundancy="dci", lam_bar=None, return_log=False):
    m = load_model()
    d = mujoco.MjData(m)
    dt = m.opt.timestep
    pen = m.site("pen").id
    body = m.site_bodyid[pen]
    d.qpos[:] = Q_INIT
    mujoco.mj_forward(m, d)
    x0 = d.site_xpos[pen].copy()
    if lam_bar is None:
        lam_bar = np.linalg.eigvalsh(Arm(m).Lambda(Q_INIT)).max()
    law = ImpedanceLaw("const", lam=alpha * lam_bar, D=D)
    ctrl = VariableImpedanceController(m, law, redundancy, K_I=K_I)
    queue = deque([d.qfrc_bias.copy()] * delay_steps)  # latency buffer starts holding the arm

    n = int(T / dt)
    v = np.zeros((n, 3))
    xs = np.zeros((n, 3))
    diverged = False
    for k in range(n):
        t = k * dt
        x = d.site_xpos[pen].copy()
        # human: shaking target, grip stiffness varying smoothly between k_lo and k_hi (2 Hz)
        tgt = x0 + amp * np.array([np.sin(2 * np.pi * f_shake * t),
                                   np.sin(2 * np.pi * f_shake * t + 2.1),
                                   np.sin(2 * np.pi * f_shake * t + 4.2)])
        kh = k_lo + (k_hi - k_lo) * (0.5 - 0.5 * np.cos(2 * np.pi * t / 0.5))
        f = kh * (tgt - x)
        tau = ctrl(d.qpos.copy(), d.qvel.copy())
        queue.append(tau)
        d.ctrl[:] = queue.popleft()
        d.qfrc_applied[:] = 0
        mujoco.mj_applyFT(m, d, f, np.zeros(3), x, body, d.qfrc_applied)
        mujoco.mj_step(m, d)
        v[k], xs[k] = ctrl.arm.xd, x
        if not np.all(np.isfinite(d.qvel)) or np.abs(d.qvel).max() > 50:
            diverged = True
            v, xs = v[:k + 1], xs[:k + 1]
            break

    # vibration index: RMS of the high-frequency (>15 Hz) part of the pen velocity
    if diverged:
        vib = np.inf
    else:
        seg = v[len(v) // 3:]
        F = np.fft.rfft(seg - seg.mean(0), axis=0)
        freqs = np.fft.rfftfreq(len(seg), dt)
        F[freqs < 15.0] = 0
        vib = np.sqrt(np.mean(np.sum(np.fft.irfft(F, n=len(seg), axis=0) ** 2, axis=1)))
    if return_log:
        return vib, diverged, dict(t=np.arange(len(v)) * dt, v=v, x=xs)
    return vib, diverged


def sweep(alphas, Ds, threshold=0.05, **kw):
    lam_bar = np.linalg.eigvalsh(Arm(load_model()).Lambda(Q_INIT)).max()
    vib = np.zeros((len(alphas), len(Ds)))
    for i, a in enumerate(alphas):
        for j, D in enumerate(Ds):
            vib[i, j], _ = shake_test(a, D, lam_bar=lam_bar, **kw)
            print(f"alpha={a:5.3f}  D={D:5.1f}  vib={vib[i, j]:.4g}  {'UNSTABLE' if vib[i, j] > threshold else 'ok'}",
                  flush=True)
    return vib, lam_bar


def plot(alphas, Ds, vib, lam_bar, threshold, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    stable = vib <= threshold
    A, DD = np.meshgrid(alphas, Ds, indexing="ij")
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.3))
    ax[0].scatter(DD[stable], A[stable], c="tab:green", marker="o", label="stable")
    ax[0].scatter(DD[~stable], A[~stable], c="tab:red", marker="x", s=50, label="vibrating / unstable")
    ax[0].set_xlabel("damping D [N s/m]")
    ax[0].set_ylabel(r"inertia scaling $\alpha$  ($\lambda_d=\alpha\bar\lambda$)")
    ax[0].set_title(fr"Stability region ($\bar\lambda$ = {lam_bar:.2f} kg)")
    ax[0].legend(loc="center right")
    ax[0].grid(alpha=0.3)

    lam = A * lam_bar
    Tc = lam / DD
    ax[1].scatter(DD[stable], Tc[stable], c="tab:green", marker="o", label="stable")
    ax[1].scatter(DD[~stable], Tc[~stable], c="tab:red", marker="x", s=50, label="vibrating / unstable")
    # minimum stable constant-mass locus
    lam_min = np.nanmax(np.where(~stable, lam, np.nan)) if (~stable).any() else None
    Dl = np.linspace(min(Ds), max(Ds), 200)
    if lam_min is not None:
        ax[1].plot(Dl, lam_min / Dl, "k--", label=fr"largest unstable $\lambda_d$ = {lam_min:.2f} kg")
    from vic import time_constant_L4
    ax[1].plot(Dl, time_constant_L4(Dl), "b-.", label="paper L4 min-T curve")
    ax[1].set_yscale("log")
    ax[1].set_xlabel("damping D [N s/m]")
    ax[1].set_ylabel(r"time constant $T=\lambda_d/D$ [s]")
    ax[1].set_title("Same region in the (D, T) plane")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print("saved", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--delay", type=int, default=2, help="torque latency [steps of 1 ms]")
    ap.add_argument("--K_I", type=float, default=100.0)
    ap.add_argument("--threshold", type=float, default=0.05, help="vibration RMS threshold [m/s]")
    a = ap.parse_args()
    alphas = np.array([1.0, 0.7, 0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.12, 0.1, 0.08, 0.06, 0.04])
    Ds = np.array([5, 10, 20, 30, 40, 50, 60], float)
    vib, lam_bar = sweep(alphas, Ds, a.threshold, delay_steps=a.delay, K_I=a.K_I)
    out_dir = os.path.join(HERE, "results")
    os.makedirs(out_dir, exist_ok=True)
    np.savez(os.path.join(out_dir, "stability_region.npz"), alphas=alphas, Ds=Ds, vib=vib, lam_bar=lam_bar)
    plot(alphas, Ds, vib, lam_bar, a.threshold, os.path.join(out_dir, "stability_region.png"))


if __name__ == "__main__":
    main()
