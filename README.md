# Variable Impedance Control of a Redundant Manipulator in MuJoCo (KUKA LBR iiwa 14)

ENPM 667 Project 1. This repo is a MuJoCo reproduction and study of

> F. Ficuciello, L. Villani, B. Siciliano, **"Variable Impedance Control of Redundant Manipulators for Intuitive Human–Robot Physical Interaction,"** *IEEE Transactions on Robotics*, 31(4):850–863, 2015. doi:10.1109/TRO.2015.2430053

The robot is the standard **KUKA LBR iiwa 14** model from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/kuka_iiwa_14), the successor of the KUKA LWR4 used in the paper (same 7-DOF kinematic structure and joint sign convention). It holds a paint marker. A simulated human guides it along a drawing on a desk (the paper's cooperative writing task). The arm runs the paper's controller:

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
mjpython writing_sim.py --view --redundancy none                 # joint 6 runs into its limit

# reproduce every figure/table in the report  (results/)
python sim_1dof.py        # Sec. VI 1-DOF study      (~10 s)
python stability.py       # Sec. V stability region  (~1-2 min)
python experiments.py     # Sec. VII writing task    (~3-10 min, uses all cores)
python render_videos.py   # results/videos/*.mp4     (~2 min)
```

In the viewer, the black line is the path the human wants to draw, the red line is the pen trace, and the green ball is the point the human's hand is pulling toward.


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
| `models/kuka_iiwa_14/` | KUKA LBR iiwa 14 from MuJoCo Menagerie, unmodified (BSD-3-Clause, see its LICENSE) |
| `vic.py` | `load_model()` (torque motors, marker, desk) and the controller: `Arm` (Λ, J̄, null-space projector), `MomentumObserver`, `ImpedanceLaw` (const / variable D / L1–L4), `Redundancy` (manipulability, DCI), `VariableImpedanceController` (eqs. 9–10) |
| `task.py` | writing path (straight lines, 3 sharp corners, a semicircle) and the human-operator model |
| `writing_sim.py` | one writing-task run; CLI, metrics (H, e, path deviation), viewer, video |
| `sim_1dof.py` | paper Sec. VI: 1-DOF comparison of damping/mass laws (Figs. 4, 6, 7) |
| `stability.py` | paper Sec. V: stability region in the (α, D) plane (Figs. 2, 3) |
| `experiments.py` | paper Sec. VII: redundancy vs stability/performance, laws L1–L4, variable vs constant + t-test |
| `render_videos.py` | renders the mp4 clips |
| `results/` | generated figures, `results.md` tables, `results.json`, videos |
