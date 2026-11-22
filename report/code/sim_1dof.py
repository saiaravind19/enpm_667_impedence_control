"""
1-DOF simulation study of the impedance modulation laws (paper Sec. VI, Figs. 4, 6, 7).

Robot end effector along one Cartesian direction with the imposed impedance (eq. 4)
        lambda(x') x'' + D(x') x' = f
driven by a human arm modeled as a pure stiffness (Fig. 5)
        f = k (x_d - x),   k = 200 N/m,
where x_d(t) is a raised-cosine reference: 0.2 m in 4 s.

    python sim_1dof.py      # saves results/fig4_T_vs_D.png, fig6_low_mass.png, fig7_high_mass.png
"""

from __future__ import annotations

import os

import numpy as np

from vic import T_L3, damping_law, time_constant_L4

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")


def raised_cosine(t, L=0.2, Tm=4.0, t0=0.5):
    tau = np.clip((t - t0) / Tm, 0, 1)
    return L * (1 - np.cos(np.pi * tau)) / 2


def simulate(mass_law, damp_law, k=200.0, T=6.0, dt=1e-4):
    n = int(T / dt)
    t = np.arange(n) * dt
    x = v = 0.0
    X, V, F, LAM, DD = (np.zeros(n) for _ in range(5))
    for i in range(n):
        f = k * (raised_cosine(t[i]) - x)
        D = damp_law(v)
        lam = mass_law(D)
        a = (f - D * v) / lam
        v += a * dt  # semi-implicit Euler
        x += v * dt
        X[i], V[i], F[i], LAM[i], DD[i] = x, v, f, lam, D
    return dict(t=t, x=X, v=V, f=F, lam=LAM, D=DD, xd=raised_cosine(t))


def var_D(v):
    return float(damping_law(np.array([v]))[0])


MASS_LAWS = {
    "L1: λ = 1.1 kg": lambda D: 1.1,
    "L2: λ = 5.6 kg": lambda D: 5.6,
    "L3: const T": lambda D: D * T_L3,
    "L4: min T(D)": lambda D: D * time_constant_L4(D),
}


def metrics(r):
    t, v, f = r["t"], r["v"], r["f"]
    late = t > 4.5 - 0.0  # end of the motion (reference stops at t = 4.5 s)
    sign_changes = int(np.sum(np.diff(np.sign(v[late & (np.abs(v) > 1e-4)])) != 0))
    return dict(f_peak=np.abs(f).max(), f_rms=np.sqrt(np.mean(f[t < 4.5] ** 2)),
                v_peak=np.abs(v).max(), track_rms=np.sqrt(np.mean((r["xd"] - r["x"]) ** 2)),
                overshoot=max(0.0, r["x"].max() - 0.2), v_sign_changes=sign_changes)


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUT, exist_ok=True)

    # ---------------- Fig. 4: time-constant laws vs damping ----------------
    D = np.linspace(5, 60, 300)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(D, 1.1 / D, label="L1: λ = 1.1 kg")
    ax.plot(D, 5.6 / D, label="L2: λ = 5.6 kg")
    ax.plot(D, np.full_like(D, T_L3), label=f"L3: T = {T_L3:.3f} s")
    ax.plot(D, time_constant_L4(D), "-.", label="L4: min T(D)")
    ax.plot(D, 0.764 / D, "k:", label="λ = 0.764 kg (paper's min-mass frontier)")
    ax.set_xlabel("damping D [N s/m]")
    ax.set_ylabel("time constant T = λ/D [s]")
    ax.set_yscale("log")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8)
    ax.set_title("Impedance modulation laws (paper Fig. 4)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig4_T_vs_D.png"), dpi=150)
    plt.close(fig)

    # ---------------- Figs. 6-7: variable vs constant damping ----------------
    rows = []
    for lam, tag in [(1.1, "fig6_low_mass"), (5.6, "fig7_high_mass")]:
        cases = {
            "variable D (eq. 20)": simulate(lambda D: lam, var_D),
            "D = 5 N s/m": simulate(lambda D: lam, lambda v: 5.0),
            "D = 60 N s/m": simulate(lambda D: lam, lambda v: 60.0),
        }
        fig, ax = plt.subplots(3, 1, figsize=(7, 8), sharex=True)
        for (name, r), c in zip(cases.items(), ["C3", "C0", "C2"]):
            ax[0].plot(r["t"], r["f"], c, label=name)
            ax[1].plot(r["t"], r["v"], c, label=name)
            ax[2].plot(r["t"], r["D"], c, label=name)
            rows.append((f"λ={lam}", name, metrics(r)))
        ax[1].plot(r["t"], np.gradient(r["xd"], r["t"]), "k--", lw=1, label="reference velocity")
        ax[0].set_ylabel("human force f [N]")
        ax[1].set_ylabel("velocity [m/s]")
        ax[2].set_ylabel("damping D [N s/m]")
        ax[2].set_xlabel("time [s]")
        for a_ in ax:
            a_.grid(alpha=0.3)
        ax[0].legend(fontsize=8)
        ax[1].legend(fontsize=8)
        ax[0].set_title(f"1-DOF human guidance, λ = {lam} kg (paper Fig. {6 if lam < 2 else 7})")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"{tag}.png"), dpi=150)
        plt.close(fig)

    # ---------------- mass laws L1-L4 with variable damping ----------------
    fig, ax = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    for name, law in MASS_LAWS.items():
        r = simulate(law, var_D)
        ax[0].plot(r["t"], r["f"], label=name)
        ax[1].plot(r["t"], r["v"], label=name)
        rows.append(("var D", name, metrics(r)))
    ax[0].set_ylabel("human force f [N]")
    ax[1].set_ylabel("velocity [m/s]")
    ax[1].set_xlabel("time [s]")
    ax[0].legend(fontsize=8)
    for a_ in ax:
        a_.grid(alpha=0.3)
    ax[0].set_title("Variable damping with mass laws L1-L4")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_1dof_mass_laws.png"), dpi=150)
    plt.close(fig)

    lines = ["| case | law | peak force [N] | RMS force [N] | peak vel [m/s] | RMS tracking err [mm] | overshoot [mm] | vel. sign changes at stop |",
             "|---|---|---|---|---|---|---|---|"]
    for a_, b_, mt in rows:
        lines.append(f"| {a_} | {b_} | {mt['f_peak']:.2f} | {mt['f_rms']:.2f} | {mt['v_peak']:.3f} | "
                     f"{1000 * mt['track_rms']:.1f} | {1000 * mt['overshoot']:.2f} | {mt['v_sign_changes']} |")
    table = "\n".join(lines)
    open(os.path.join(OUT, "table_1dof.md"), "w").write(table + "\n")
    print(table)


if __name__ == "__main__":
    main()
