# Variable Impedance Control of a Redundant Manipulator in MuJoCo

ENPM 667 Project 1. This repo is a MuJoCo reproduction and study of

> F. Ficuciello, L. Villani, B. Siciliano, **"Variable Impedance Control of Redundant Manipulators for Intuitive Human–Robot Physical Interaction,"** *IEEE Transactions on Robotics*, 31(4):850–863, 2015. doi:10.1109/TRO.2015.2430053

A 7-DOF KUKA LWR4-like arm holds a pen. A simulated human guides it along a drawing on a desk (the paper's cooperative writing task). The arm runs the paper's controller:

- **Cartesian impedance control with inertia reshaping** (eq. 9). The end effector behaves as `Λd ẍ + Dd ẋ = F_ext`.
- **External-force estimation** with a generalized-momentum observer (eqs. 6–8). No force sensor is used.
- **Redundancy resolution** in the dynamically consistent null space (eq. 10). The secondary task either maximizes manipulability (eqs. 11–12) or minimizes the dynamic conditioning index (DCI) of the end-effector inertia (eqs. 14–18).
- **Variable impedance.** Damping depends on the pen velocity, `D(ẋ) = max(60 e^{-4|ẋ|}, 5)` (eq. 20), combined with mass laws L1–L4.

## Quick start (macOS / Linux)

```bash
cd ~/projects/impedance_mujoco
uv venv --python 3.12 .venv && source .venv/bin/activate   # or: python3 -m venv .venv
uv pip install -r requirements.txt                         # or: pip install -r requirements.txt

# live 3-D viewer (macOS needs mjpython for the passive viewer)
mjpython writing_sim.py --view                                   # variable impedance L1 + DCI
mjpython writing_sim.py --view --law const --D 60                # constant high damping
mjpython writing_sim.py --view --law const --D 20                # constant low damping
mjpython writing_sim.py --view --low-inertia --redundancy manip  # goes unstable
mjpython writing_sim.py --view --low-inertia --redundancy dci    # stays stable

# reproduce every figure/table in the report  (results/)
python sim_1dof.py        # Sec. VI 1-DOF study      (~10 s)
python stability.py       # Sec. V stability region  (~1-2 min)
python experiments.py     # Sec. VII writing task    (~3-10 min, uses all cores)
python render_videos.py   # results/videos/*.mp4     (~2 min)
```

In the viewer, the black line is the path the human wants to draw, the red line is the pen trace, and the green ball is the point the human's hand is pulling toward.

## Results at a glance

(5 simulated subjects per condition, DCI null-space task; full tables in `results/results.md`)

| impedance | time H [s] | mean path deviation [mm] | mean human force [N] |
|---|---|---|---|
| variable damping (L1) | 15.14 | 1.31 | 2.37 |
| constant high damping (D = 60) | 16.20 | 1.36 | 2.71 |
| constant low damping (D = 20) | 12.99 | 1.99 | 1.48 |

- Variable damping is significantly faster than high damping (p = 0.001) with no significant accuracy loss, and significantly more accurate than low damping (p < 0.001), as in the paper's Table I.
- With low virtual inertia, the DCI null-space task stays stable while manipulability maximization diverges (paper Sec. VII-A).
- The (α, D) stability region: stable for all D ∈ [5, 60] N·s/m when α ≥ 0.3, unstable for all D when α ≤ 0.06 (paper: 0.25 and 0.1).

## Videos (`results/videos/`)

Left: 3/4 view. Right: top-down view of the sheet (arm hidden).

| clip | what it shows |
|---|---|
| `main.mp4` | variable impedance L1 + DCI, full drawing |
| `hconst.mp4` | constant high damping, full drawing |
| `manip_low.mp4` | low virtual inertia + manipulability task: becomes unstable |
| `dci_low.mp4` | low virtual inertia + DCI task: completes |

## Files

| file | what it is |
|---|---|
| `lwr4.xml` | MJCF model: 7-DOF arm with LWR4 link lengths and joint-axis convention, paint marker, desk, A4 sheet |
| `vic.py` | the controller: `Arm` (Λ, J̄, null-space projector), `MomentumObserver`, `ImpedanceLaw` (const / variable D / L1–L4), `Redundancy` (manipulability, DCI), `VariableImpedanceController` (eqs. 9–10) |
| `task.py` | writing path (straight lines, 3 sharp corners, a semicircle) and the human-operator model |
| `writing_sim.py` | one writing-task run; CLI, metrics (H, e, path deviation), viewer, video |
| `sim_1dof.py` | paper Sec. VI: 1-DOF comparison of damping/mass laws (Figs. 4, 6, 7) |
| `stability.py` | paper Sec. V: stability region in the (α, D) plane (Figs. 2, 3) |
| `experiments.py` | paper Sec. VII: redundancy vs stability/performance, laws L1–L4, variable vs constant + t-test |
| `render_videos.py` | renders the mp4 clips |
| `results/` | generated figures, `results.md` tables, `results.json`, videos |

## Modelling choices (and how they differ from the paper)

- **Robot.** No identified LWR4 dynamic model is used. The arm has the LWR4 link lengths and joint-axis signs, with approximate link masses (about 15 kg total) and 0.005 kg·m² armature on the wrist joints. At the paper's start pose `q_i = [2.35 22.8 −1.54 −53.2 −3.1 101.15 0]°`, the pen points straight down. The apparent inertia there has eigenvalues 0.26 / 0.32 / 2.41 kg; the paper reports 0.12 / 0.20 / 2.95 kg.
- **Joint friction.** Viscous friction of 0.5 N·m·s/rad is present in the simulated robot. The controller does not model it, so the momentum observer picks it up, as on the real robot.
- **Latency.** Torque commands reach the joints 2 ms late, like the LWR4's FRI interface. Without this latency no case in the stability sweep goes unstable, even at the smallest virtual inertia.
- **Human.** Following the paper's Fig. 5, the hand is a spring, `f = k(x_h − x)` with k = 200 N/m. It pulls the pen toward a point `x_h` that moves along the drawing. The hand slows near corners and on the arc, waits if the pen lags more than 4 cm, and adds band-limited force noise (0.8 N, 4 Hz) plus 9 Hz tremor (0.3 N). Five "subjects" use different noise seeds and preferred speeds (×0.9 to ×1.1).
- **Coriolis term.** Eq. (9) lumps C q̇ into the residual r. Here the observer is built so that r → τ_ext − τ_f, and C q̇ + g is compensated from the model. Once the observer has converged, the two are equivalent.
- **Eq. (20).** The paper prints `min{a e^{−b|ẋ|}, 5}`; the text describes a lower saturation at 5 N·s/m, so `max` is used.

## Requirements

Python ≥ 3.10, `mujoco` ≥ 3.1, `numpy`, `scipy`, `matplotlib`, `imageio`, `imageio-ffmpeg`.
