"""
Runs every writing-task experiment of the paper (Sec. VII) in MuJoCo and writes the
figures/tables used in the report to results/.

    python experiments.py            # ~10 min on a laptop (uses all cores)

A1  Redundancy vs stability   (paper VII-A, Figs. 10-11)
A2  Redundancy vs performance (paper VII-B, Fig. 12)
A3  Variable impedance laws L1-L4 (paper VII-C, Fig. 13)
A4  Variable vs constant impedance + t-test (paper VII-D, Figs. 14-16, Table I)
A5  Momentum-observer force estimate vs true human force
"""

from __future__ import annotations

import json
import os
from multiprocessing import Pool

import numpy as np

from vic import ImpedanceLaw

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")

# five simulated "subjects": different hand-noise realizations and preferred speeds
SUBJECTS = [dict(seed=i, speed_scale=s) for i, s in enumerate([1.0, 0.9, 1.1, 0.95, 1.05])]

LAMBDA_DIAG_QI = None  # filled in main(): diag of Lambda(q_i)


def _job(args):
    from writing_sim import run
    name, law, red, subj, keep = args
    log, res, path = run(law, red, verbose=False, **subj)
    res.update(name=name, subject=subj["seed"], redundancy=red)
    print(f"  {name:28s} {red:5s} subj {subj['seed']}  H={res['H']:6.2f}  e={1000 * res['e']:6.1f} mm  "
          f"dev={1000 * res['dev_mean']:.2f} mm{'  UNSTABLE' if res['unstable'] else ''}", flush=True)
    if keep:
        sub = {k: v[::5] for k, v in log.items()}  # 200 Hz is plenty for plots
        return res, sub
    return res, None


def run_all(jobs, procs=None):
    with Pool(procs or os.cpu_count()) as p:
        return p.map(_job, jobs)


def summarize(results, key="name"):
    groups = {}
    for r in results:
        groups.setdefault((r[key], r["redundancy"]), []).append(r)
    rows = []
    for (n, red), rs in groups.items():
        H = np.array([r["H"] for r in rs])
        e = np.array([r["e"] for r in rs]) * 1000
        dv = np.array([r["dev_mean"] for r in rs]) * 1000
        f = np.array([r["f_mean"] for r in rs])
        rows.append(dict(name=n, redundancy=red, n=len(rs), unstable=sum(r["unstable"] for r in rs),
                         H_mean=np.nanmean(H) if np.isfinite(H).any() else np.nan, H_std=np.nanstd(H),
                         e_mean=e.mean(), e_std=e.std(), dev_mean=dv.mean(), dev_std=dv.std(),
                         f_mean=f.mean(), dci_mean=np.mean([r["dci_mean"] for r in rs])))
    return rows


def md_table(rows):
    s = ["| impedance | null-space task | runs | unstable | H [s] | e [mm] | mean path dev. [mm] | mean |f| [N] | mean DCI |",
         "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        s.append(f"| {r['name']} | {r['redundancy']} | {r['n']} | {r['unstable']} | {r['H_mean']:.2f} ± {r['H_std']:.2f} | "
                 f"{r['e_mean']:.1f} ± {r['e_std']:.1f} | {r['dev_mean']:.2f} ± {r['dev_std']:.2f} | "
                 f"{r['f_mean']:.2f} | {r['dci_mean']:.2f} |")
    return "\n".join(s)


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import mujoco
    from scipy import stats

    from task import WritingPath
    from vic import Q_INIT, load_model, Arm

    os.makedirs(OUT, exist_ok=True)
    arm = Arm(load_model())
    Lqi = arm.Lambda(Q_INIT)
    dq = np.diag(Lqi)
    summary = {"Lambda_qi": Lqi.tolist(), "Lambda_qi_eig": np.linalg.eigvalsh(Lqi).tolist()}
    report = []

    L1 = ImpedanceLaw("L1")
    HCONST = ImpedanceLaw("const", lam=1.1, D=60.0, name="const high D (λ=1.1, D=60)")
    LCONST = ImpedanceLaw("const", lam=1.1, D=20.0, name="const low D (λ=1.1, D=20)")
    s0 = SUBJECTS[0]

    # ------------------------------------------------------------------ A1
    print("A1 redundancy vs stability")
    levels = [1.0, 1.5, 2.0, 3.0]
    jobsA1 = []
    for lv in levels:
        law = ImpedanceLaw("var", lam=lv * 0.27 * dq, name=f"var D, Λd = {lv:g}×low")
        for red in ["manip", "dci"]:
            jobsA1.append((law.name, law, red, s0, lv in (1.0, 1.5)))
    outA1 = run_all(jobsA1)
    resA1 = [o[0] for o in outA1]
    report.append("## A1 Redundancy vs stability (low virtual inertia, variable damping)\n")
    report.append(f"Low inertia Λd,low = 0.27·diag(Λ(q_i)) = {np.round(0.27 * dq, 3).tolist()} kg "
                  "(same ratio as the paper's diag{0.0328, 0.0548, 0.8138}).\n")
    report.append(md_table(summarize(resA1)))

    fig, ax = plt.subplots(2, 2, figsize=(11, 7.5))
    path = WritingPath(Q_INIT[:2] * 0, 0)  # placeholder, replaced below
    for col, lv in enumerate([1.0, 1.5]):
        for (res, log), job in zip(outA1, jobsA1):
            if log is None or not job[0].endswith(f"{lv:g}×low"):
                continue
            red = job[2]
            c = "C3" if red == "manip" else "C0"
            lab = "manipulability (Man)" if red == "manip" else "DCI"
            if res["unstable"]:
                lab += " – unstable"
            ax[0, col].plot(log["x"][:, 1], log["x"][:, 0], c, lw=1, label=lab)
            ax[1, col].plot(log["t"], log["dci"], c, label=lab)
            path = res.get("_path", path)
        ax[0, col].set_title(f"Pen path, Λd = {lv:g} × low inertia")
        ax[1, col].set_title(f"DCI ω(q), Λd = {lv:g} × low inertia")
        ax[1, col].set_xlabel("time [s]")
        ax[1, col].set_ylabel("ω(q)")
        ax[0, col].set_xlabel("y [m]")
        ax[0, col].set_ylabel("x [m]")
        ax[0, col].set_aspect("equal")
        ax[1, col].set_yscale("log")
        for a_ in ax[:, col]:
            a_.grid(alpha=0.3)
            a_.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A1_redundancy_stability.png"), dpi=150)
    plt.close(fig)

    # ------------------------------------------------------------------ A2 / A3 / A4
    print("A2-A4 performance experiments (5 subjects each)")
    jobs = []
    for red in ["none", "manip", "dci"]:
        for law in [HCONST, L1]:
            for s in SUBJECTS:
                jobs.append((law.label, law, red, s, s is s0 and red == "dci"))
    for law in [ImpedanceLaw("L2"), ImpedanceLaw("L3"), ImpedanceLaw("L4"), LCONST]:
        for s in SUBJECTS:
            jobs.append((law.label, law, "dci", s, s is s0))
    out = run_all(jobs)
    res_all = [o[0] for o in out]
    logs = {(j[0], j[2]): o[1] for j, o in zip(jobs, out) if o[1] is not None}
    rows = summarize(res_all)
    report.append("\n\n## A2-A4 Writing task, 5 simulated subjects (mean ± std)\n")
    report.append(md_table(rows))
    report.append("\nNull-space task 'none' (damping only): the self-motion drives joint 6 into its "
                  "+120 deg limit after ~4.5 s and the drawing is never completed (H = nan), so its "
                  "e and path deviation describe a failed run, not accuracy.")
    summary["rows"] = rows

    def pick(name, red):
        return [r for r in res_all if r["name"] == name and r["redundancy"] == red]

    def scatter(ax, groups, title):
        for (name, red, lab, c, mk) in groups:
            rs = pick(name, red)
            H = np.array([r["H"] for r in rs])
            e = np.array([r["e"] for r in rs]) * 1000
            ax.scatter(H, e, c=c, marker=mk, alpha=0.55, s=30)
            ax.scatter(np.nanmean(H), e.mean(), c=c, marker=mk, s=160, edgecolors="k", label=lab)
        ax.set_xlabel("execution time H [s]")
        ax.set_ylabel("length error e [mm]")
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)

    # A2 (Fig. 12)
    fig, ax = plt.subplots(figsize=(6.5, 4.8))
    scatter(ax, [(HCONST.label, "manip", "const, Man", "C3", "s"),
                 (HCONST.label, "dci", "const, DCI", "C0", "s"),
                 (L1.label, "manip", "variable, Man", "C3", "o"),
                 (L1.label, "dci", "variable, DCI", "C0", "o")],
            "Redundancy resolution vs performance (paper Fig. 12)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A2_redundancy_performance.png"), dpi=150)
    plt.close(fig)

    # A3 (Fig. 13)
    fig, ax = plt.subplots(figsize=(6.5, 4.8))
    scatter(ax, [(ImpedanceLaw(k).label, "dci", ImpedanceLaw(k).label, c, "o")
                 for k, c in zip(["L1", "L2", "L3", "L4"], ["C0", "C1", "C2", "C4"])],
            "Variable impedance laws L1–L4, DCI (paper Fig. 13)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A3_laws.png"), dpi=150)
    plt.close(fig)

    # A4 (Fig. 15) + t-test (Table I)
    fig, ax = plt.subplots(figsize=(6.5, 4.8))
    scatter(ax, [(L1.label, "dci", "variable (L1)", "C0", "o"),
                 (HCONST.label, "dci", "const high D = 60", "C2", "s"),
                 (LCONST.label, "dci", "const low D = 20", "C3", "D")],
            "Variable vs constant impedance, DCI (paper Fig. 15)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A4_var_vs_const.png"), dpi=150)
    plt.close(fig)

    groups = {"var": pick(L1.label, "dci"), "hconst": pick(HCONST.label, "dci"), "lconst": pick(LCONST.label, "dci")}
    ttab = ["| metric | var vs hconst | var vs lconst | hconst vs lconst |", "|---|---|---|---|"]
    tt = {}
    for metric, key in [("time H", "H"), ("length error e", "e"), ("mean path deviation", "dev_mean"),
                        ("mean force", "f_mean")]:
        cells = []
        for a_, b_ in [("var", "hconst"), ("var", "lconst"), ("hconst", "lconst")]:
            A = np.array([r[key] for r in groups[a_]])
            B = np.array([r[key] for r in groups[b_]])
            t, p = stats.ttest_rel(A, B)  # same 5 subjects -> paired
            sig = p < 0.05
            rel = "<" if A.mean() < B.mean() else ">"
            cells.append(f"{a_}{rel}{b_}, p = {p:.3g}{' (h=1)' if sig else ' (h=0)'}")
            tt[f"{key}:{a_}-{b_}"] = dict(t=float(t), p=float(p))
        ttab.append(f"| {metric} | " + " | ".join(cells) + " |")
    report.append("\n\n## A4 Paired t-test, variable vs constant impedance (5 subjects; h=1 ↔ p<0.05)\n")
    report.append("\n".join(ttab))
    summary["ttest"] = tt

    # path plots (Fig. 10 analog) + damping history (Fig. 14) + force norm (Fig. 16)
    import mujoco as mj
    m = load_model()
    d = mj.MjData(m)
    d.qpos[:] = Q_INIT
    mj.mj_forward(m, d)
    x0 = d.site_xpos[m.site("pen").id]
    path = WritingPath(x0[:2], x0[2])

    fig, ax = plt.subplots(1, 3, figsize=(14, 4.8), sharey=True)
    for a_, (lab, name) in zip(ax, [("variable (L1)", L1.label), ("const high D = 60", HCONST.label),
                                     ("const low D = 20", LCONST.label)]):
        lg = logs[(name, "dci")]
        a_.plot(path.xyz[:, 1], path.xyz[:, 0], "k--", lw=1.2, label="reference path")
        a_.plot(lg["x"][:, 1], lg["x"][:, 0], "C3", lw=1, label="pen")
        a_.set_title(lab)
        a_.set_xlabel("y [m]")
        a_.set_aspect("equal")
        a_.grid(alpha=0.3)
    ax[0].set_ylabel("x [m]")
    ax[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A4_paths.png"), dpi=150)
    plt.close(fig)

    lg = logs[(L1.label, "dci")]
    fig, ax = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    for i, c in enumerate("xyz"):
        ax[0].plot(lg["t"], lg["D"][:, i], label=f"D_{c}")
    ax[0].set_ylabel("damping [N s/m]")
    ax[0].legend(fontsize=8)
    ax[0].set_title("Variable damping during the task, L1 (paper Fig. 14)")
    ax[1].plot(lg["t"], np.linalg.norm(lg["xdot"][:, :2], axis=1), "k")
    ax[1].set_ylabel("pen speed [m/s]")
    ax[1].set_xlabel("time [s]")
    for a_ in ax:
        a_.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A4_damping_history.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 3.8))
    for lab, name, c in [("high D", HCONST.label, "C2"), ("variable", L1.label, "C0"), ("low D", LCONST.label, "C3")]:
        lg = logs[(name, "dci")]
        fn = np.linalg.norm(lg["f"][:, :2], axis=1)
        ax.plot(lg["t"], fn, c, lw=0.8, label=f"{lab} (mean {fn.mean():.2f} N)")
        ax.axhline(fn.mean(), color=c, ls="--", lw=1)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("|f| human force [N]")
    ax.set_title("Force exerted by the operator (paper Fig. 16)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A4_forces.png"), dpi=150)
    plt.close(fig)

    # ------------------------------------------------------------------ A5 observer
    lg = logs[(L1.label, "dci")]
    fig, ax = plt.subplots(3, 1, figsize=(8, 6.5), sharex=True)
    for i, c in enumerate("xyz"):
        ax[i].plot(lg["t"], lg["f"][:, i], "k", lw=1, label="true human force")
        ax[i].plot(lg["t"], lg["F_hat"][:, i], "C1", lw=1, label="observer estimate (eq. 8)")
        ax[i].set_ylabel(f"F_{c} [N]")
        ax[i].grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    ax[0].set_title("Momentum-observer force estimate, K_I = 100 s⁻¹")
    ax[2].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "A5_observer.png"), dpi=150)
    plt.close(fig)
    rm = [r["obs_rmse"] for r in pick(L1.label, "dci")]
    report.append(f"\n\n## A5 Observer\nRMS error of F_hat vs true human force (L1, DCI, 5 subjects): "
                  f"{np.mean(rm):.3f} N (mean |f| = {np.mean([r['f_mean'] for r in pick(L1.label, 'dci')]):.2f} N)\n")

    text = "\n".join(report)
    open(os.path.join(OUT, "results.md"), "w").write(text + "\n")
    json.dump(dict(summary, A1=resA1, all=res_all), open(os.path.join(OUT, "results.json"), "w"),
              indent=1, default=float)
    print(text)


if __name__ == "__main__":
    main()
