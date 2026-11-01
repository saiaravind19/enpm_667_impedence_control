"""
Human-robot cooperative writing task in MuJoCo (paper Sec. VII).

Examples
    python writing_sim.py                                  # L1 variable impedance + DCI
    python writing_sim.py --law const --lam 1.1 --D 60     # high constant damping
    python writing_sim.py --law const --lam 1.1 --D 20     # low constant damping
    python writing_sim.py --redundancy manip               # manipulability null-space task
    python writing_sim.py --video results/writing.mp4      # render a video
    mjpython writing_sim.py --view                         # live viewer (macOS: mjpython)
    mjpython writing_sim.py --view --low-inertia --redundancy manip   # watch it go unstable
"""

from __future__ import annotations

import argparse
import os
import time

import mujoco
import numpy as np

from task import HumanOperator, WritingPath
from vic import (Q_INIT, XML, ImpedanceLaw, VariableImpedanceController, dci,
                 manipulability)

HERE = os.path.dirname(os.path.abspath(__file__))


def run(law: ImpedanceLaw, redundancy="dci", force_source="observer", T_max=40.0,
        k_h=200.0, view=False, video=None, verbose=True, K_I=100.0, k_c=1.0, k_m=20.0, seed=0,
        speed_scale=1.0, delay_steps=2, tail=0.5, fps=30):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    dt = m.opt.timestep
    pen = m.site("pen").id
    pen_body = m.site_bodyid[pen]

    d.qpos[:] = Q_INIT
    mujoco.mj_forward(m, d)
    x0 = d.site_xpos[pen].copy()
    path = WritingPath(x0[:2], x0[2])
    human = HumanOperator(path, k=k_h, seed=seed, speed_scale=speed_scale)
    from collections import deque
    # torque latency (LWR4 FRI ~ 1-2 ms); the buffer starts with the torque that holds the arm
    queue = deque([d.qfrc_bias.copy()] * delay_steps)
    ctrl = VariableImpedanceController(m, law, redundancy, K_I=K_I, force_source=force_source,
                                       k_c=k_c, k_m=k_m)

    n_max = int(T_max / dt)
    keys = ["x", "xh", "f", "F_hat", "lam", "D", "xdot"]
    log = {k: np.zeros((n_max, 3)) for k in keys}
    log["q"] = np.zeros((n_max, 7))
    log["dci"] = np.zeros(n_max)
    log["manip"] = np.zeros(n_max)
    log["t"] = np.arange(n_max) * dt
    log["dq"] = np.zeros(n_max)
    H = None
    unstable = False

    viewer = renderer = writer = None
    trail = []
    if view:
        from mujoco import viewer as mjv
        viewer = mjv.launch_passive(m, d)
        _setup_cam(viewer.cam)
        t_wall = time.time()
    if video:
        import imageio
        os.makedirs(os.path.dirname(os.path.abspath(video)), exist_ok=True)
        renderer = mujoco.Renderer(m, 720, 640)  # two 640x720 views side by side
        cam = mujoco.MjvCamera()
        _setup_cam(cam)
        cam_top = mujoco.MjvCamera()
        _setup_top_cam(cam_top, path)
        top_opt = mujoco.MjvOption()
        top_opt.geomgroup[1] = 0  # hide the arm in the top-down view, keep the pen tip
        writer = imageio.get_writer(video, fps=fps, codec="libx264", quality=8,
                                    macro_block_size=8)
        frame_every = int(round(1.0 / (fps * dt)))

    k = 0
    for k in range(n_max):
        t = k * dt
        q, qd = d.qpos.copy(), d.qvel.copy()
        x = d.site_xpos[pen].copy()

        f, xh = human.force(x, t, dt)
        tau = ctrl(q, qd, F_true=f)

        # human force applied at the pen tip
        d.qfrc_applied[:] = 0
        mujoco.mj_applyFT(m, d, f, np.zeros(3), x, pen_body, d.qfrc_applied)
        queue.append(tau)
        d.ctrl[:] = queue.popleft()

        a = ctrl.arm
        log["x"][k], log["xh"][k], log["f"][k] = x, xh, f
        log["F_hat"][k], log["lam"][k], log["D"][k] = ctrl.F_hat, ctrl.lam_d, ctrl.D_d
        log["xdot"][k], log["q"][k] = a.xd, q
        log["dci"][k] = dci(a.Lam)
        log["manip"][k] = manipulability(a.J)
        log["dq"][k] = np.abs(qd).max()

        mujoco.mj_step(m, d)

        if not np.all(np.isfinite(d.qpos)) or np.abs(d.qvel).max() > 20:
            unstable = True
            break

        if H is None and human.s >= path.length and np.linalg.norm(x - path.point(path.length)) < 0.003 \
                and np.linalg.norm(a.xd) < 0.01:
            H = t
        if H is not None and t > H + tail:
            break

        if k % 20 == 0:
            trail.append(x.copy())
        if viewer is not None:
            if not viewer.is_running():
                break
            if k % 16 == 0:
                with viewer.lock():
                    _draw(viewer.user_scn, path, trail, xh)
                viewer.sync()
                lag = t - (time.time() - t_wall)
                if lag > 0:
                    time.sleep(lag)
        if writer is not None and k % frame_every == 0:
            views = []
            for c, opt in ((cam, None), (cam_top, top_opt)):
                renderer.update_scene(d, c, scene_option=opt)
                _draw(renderer.scene, path, trail, xh, append=True)
                views.append(renderer.render())
            writer.append_data(np.hstack(views))

    n = k + 1
    for key in log:
        log[key] = log[key][:n]
    if writer is not None:
        writer.close()
        renderer.close()

    res = metrics(log, path, H, unstable)
    res["law"], res["redundancy"], res["force_source"] = law.label, redundancy, force_source
    if verbose:
        print(f"[{redundancy:5s}] {law.label:38s} H={res['H']:6.2f} s  e={1000 * res['e']:6.2f} mm  "
              f"mean dev={1000 * res['dev_mean']:5.2f} mm  max dev={1000 * res['dev_max']:5.2f} mm  "
              f"|f| mean={res['f_mean']:5.2f} N  DCI mean={res['dci_mean']:.3f}"
              + ("  UNSTABLE" if unstable else ""))
    if viewer is not None:
        print("finished - close the viewer window to exit")
        while viewer.is_running():  # keep the final drawing on screen
            with viewer.lock():
                _draw(viewer.user_scn, path, trail, xh)
            viewer.sync()
            time.sleep(0.05)
        viewer.close()
    return log, res, path


def metrics(log, path, H, unstable):
    t = log["t"]
    end = len(t) if H is None else np.searchsorted(t, H) + 1
    x = log["x"][:end]
    le = np.linalg.norm(np.diff(x[:, :2], axis=0), axis=1).sum()
    dev = path.distance_to_path(x[::10])
    return dict(
        H=H if H is not None else np.nan,
        e=abs(path.length - le),  # eq. (21)
        l_e=le, l_d=path.length,
        dev_mean=dev.mean(), dev_max=dev.max(),
        f_mean=np.linalg.norm(log["f"][:end, :2], axis=1).mean(),
        dci_mean=log["dci"][:end].mean(),
        obs_rmse=np.sqrt(np.mean(np.sum((log["F_hat"][:end] - log["f"][:end]) ** 2, axis=1))),
        unstable=unstable,
    )


# --------------------------------------------------------------------------- #
# visuals
# --------------------------------------------------------------------------- #
def _setup_cam(cam):
    cam.azimuth, cam.elevation, cam.distance = 105, -42, 1.75
    cam.lookat[:] = [0.30, 0.0, 0.42]


def _setup_top_cam(cam, path):
    """Top-down close-up of the sheet (robot side at the top of the image)."""
    c = 0.5 * (path.xyz.min(0) + path.xyz.max(0))
    cam.azimuth, cam.elevation, cam.distance = 180, -90, 0.42
    cam.lookat[:] = [c[0], c[1], path.z0]


def _add(scn, gtype, size, pos, rgba, mat=np.eye(3)):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.asarray(size, float), np.asarray(pos, float),
                        np.asarray(mat, float).flatten(), np.asarray(rgba, np.float32))
    scn.ngeom += 1


def _connector(scn, p, q, w, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                        np.asarray(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, np.asarray(p, float), np.asarray(q, float))
    scn.ngeom += 1


def _draw(scn, path, trail, xh, append=False):
    if not append:
        scn.ngeom = 0
    pts = path.xyz[::12]
    for p, q in zip(pts[:-1], pts[1:]):
        _connector(scn, p + [0, 0, 0.0005], q + [0, 0, 0.0005], 0.0012, [0.1, 0.1, 0.1, 1])
    tr = trail[-1500:]
    for p, q in zip(tr[:-1], tr[1:]):
        _connector(scn, p, q, 0.0018, [0.85, 0.1, 0.1, 1])
    _add(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.008, 0, 0], xh, [0.1, 0.7, 0.2, 0.8])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--law", default="L1", choices=["const", "L1", "L2", "L3", "L4"])
    ap.add_argument("--lam", type=float, default=1.1, help="mass for --law const [kg]")
    ap.add_argument("--D", type=float, default=60.0, help="damping for --law const [Ns/m]")
    ap.add_argument("--redundancy", default="dci", choices=["none", "manip", "dci"])
    ap.add_argument("--force", default="observer", choices=["observer", "sensor"])
    ap.add_argument("--low-inertia", action="store_true",
                    help="variable damping with low virtual inertia 0.27*diag(Lambda(q_i)) (paper Sec. VII-A)")
    ap.add_argument("--seed", type=int, default=0, help="simulated subject (hand-noise seed)")
    ap.add_argument("--delay", type=int, default=2, help="torque latency [ms]")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--video", default=None)
    a = ap.parse_args()
    law = ImpedanceLaw(a.law, lam=a.lam, D=a.D)
    if a.low_inertia:
        from vic import Arm
        dq = np.diag(Arm(mujoco.MjModel.from_xml_path(XML)).Lambda(Q_INIT))
        law = ImpedanceLaw("var", lam=0.27 * dq, name="variable D, low inertia")
    run(law, a.redundancy, a.force, view=a.view, video=a.video, seed=a.seed, delay_steps=a.delay)


if __name__ == "__main__":
    main()
