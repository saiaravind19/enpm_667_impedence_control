"""
Variable impedance control of a redundant (7-DOF) manipulator -- MuJoCo implementation of

    F. Ficuciello, L. Villani, B. Siciliano,
    "Variable Impedance Control of Redundant Manipulators for Intuitive
     Human-Robot Physical Interaction", IEEE Trans. Robotics 31(4), 2015.

Equation numbers in comments refer to the paper.

Pieces
------
* Arm             : kinematic/dynamic quantities from a *controller copy* of the MuJoCo
                    model: x, J, J'q', M, Lambda = (J M^-1 J^T)^-1 (eq. 2),
                    dynamically consistent inverse J# = M^-1 J^T Lambda (eq. 3).
* MomentumObserver: residual r -> tau_ext - tau_friction (eq. 6-7), De Luca et al.
                    External force estimate F_hat = J#^T r (eq. 8).
* ImpedanceLaw    : desired apparent inertia Lambda_d and damping D_d (eq. 4),
                    constant or variable: D(x') = max(a e^{-b|x'|}, D_min) (eq. 20),
                    mass laws L1..L4 (Sec. VI).
* Redundancy      : null-space secondary task u (eq. 10):
                    manipulability gradient (eq. 11-12) or DCI gradient (eq. 14-18).
* VariableImpedanceController : eq. (9)+(10).
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# Robot: KUKA LBR iiwa 14 from MuJoCo Menagerie (unmodified file, BSD-3-Clause),
# the successor of the KUKA LWR4 used in the paper (same 7-DOF S-R-S kinematics and
# joint sign convention).
MODEL_DIR = os.path.join(HERE, "models", "kuka_iiwa_14")
ROBOT_XML = os.path.join(MODEL_DIR, "iiwa14.xml")
PEN_LENGTH = 0.10  # paint marker mounted on the flange (attachment_site) [m]
PEN_MASS = 0.05  # [kg]
JOINT_FRICTION = 0.5  # viscous joint friction of the "real" robot, unknown to the controller [N m s/rad]

# Paper's start configuration for the writing task (degrees), Sec. VII. On the iiwa it
# also puts the marker vertically over the desk.
Q_INIT_DEG = np.array([2.35, 22.8, -1.54, -53.2, -3.1, 101.15, 0.0])
Q_INIT = np.deg2rad(Q_INIT_DEG)


def load_model(joint_friction=JOINT_FRICTION, visual=None) -> mujoco.MjModel:
    """Builds the simulation from the Menagerie iiwa14.xml:

    * the Menagerie position servos are replaced by plain torque motors (the paper's
      controller commands joint torques);
    * a 0.10 m paint marker is attached to the flange (site 'pen' at its tip);
    * viscous joint friction is added (a hardware effect the controller does not model);
    * a desk with an A4 sheet is placed just under the marker tip at q_i.
    Link masses, inertias and kinematics are the Menagerie values, untouched.
    visual=False drops the display meshes (dynamics are unchanged: every body has an
    explicit <inertial>); by default meshes are used when models/.../assets exists.
    """
    spec = mujoco.MjSpec.from_file(ROBOT_XML)
    if visual is None:
        visual = os.path.isdir(os.path.join(MODEL_DIR, "assets"))
    if not visual:
        for g in list(spec.geoms):
            if g.type == mujoco.mjtGeom.mjGEOM_MESH:
                spec.delete(g)
        for msh in list(spec.meshes):
            spec.delete(msh)

    spec.option.timestep = 0.001  # 1 kHz, as the LWR4/iiwa FRI interface
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_EULER

    # torque motors instead of the Menagerie position servos
    for a in list(spec.actuators):
        spec.delete(a)
    for i in range(1, 8):
        a = spec.add_actuator()
        a.name, a.target = f"motor{i}", f"joint{i}"
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.gainprm[0] = 1.0
        a.ctrllimited = 0
    for j in spec.joints:
        try:
            j.damping = joint_friction
        except TypeError:  # newer MuJoCo stores joint damping as a 3-vector
            j.damping = [joint_friction, 0.0, 0.0]

    # paint marker on the flange; display group 2 like the arm, tip in group 1
    pen = spec.body("link7").add_body()
    pen.name = "pen"
    pen.pos = [0, 0, 0.045]  # Menagerie 'attachment_site'
    g = pen.add_geom()
    g.type, g.fromto, g.size = mujoco.mjtGeom.mjGEOM_CYLINDER, [0, 0, 0, 0, 0, PEN_LENGTH - 0.006], [0.009, 0, 0]
    g.mass, g.rgba, g.group, g.contype, g.conaffinity = PEN_MASS, [0.1, 0.3, 0.9, 1], 2, 0, 0
    g = pen.add_geom()
    g.type, g.pos, g.size = mujoco.mjtGeom.mjGEOM_SPHERE, [0, 0, PEN_LENGTH - 0.006], [0.006, 0, 0]
    g.mass, g.rgba, g.group, g.contype, g.conaffinity = 0.0, [0.05, 0.05, 0.05, 1], 1, 0, 0
    s = pen.add_site()
    s.name, s.pos, s.size, s.rgba = "pen", [0, 0, PEN_LENGTH], [0.004, 0, 0], [1, 0, 0, 1]

    # pen tip at q_i decides where the desk goes
    m = spec.compile()
    d = mujoco.MjData(m)
    d.qpos[:] = Q_INIT
    mujoco.mj_forward(m, d)
    px, py, pz = d.site_xpos[m.site("pen").id]

    # scene (visual only: nothing here collides)
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1280, 720
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    tex = spec.add_texture()
    tex.name, tex.type, tex.builtin = "grid", mujoco.mjtTexture.mjTEXTURE_2D, mujoco.mjtBuiltin.mjBUILTIN_CHECKER
    tex.rgb1, tex.rgb2, tex.width, tex.height = [0.82, 0.82, 0.85], [0.74, 0.74, 0.78], 512, 512
    mat = spec.add_material()
    mat.name, mat.texrepeat, mat.reflectance = "grid", [6, 6], 0.05
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    wb = spec.worldbody
    light = wb.add_light()
    light.pos, light.dir, light.castshadow = [0.4, -0.6, 2.2], [-0.2, 0.3, -1], True

    def box(name, size, pos, rgba=None, material=None, gtype=mujoco.mjtGeom.mjGEOM_BOX):
        g = wb.add_geom()
        g.name, g.type, g.size, g.pos = name, gtype, size, pos
        g.contype = g.conaffinity = 0
        if material:
            g.material = material
        else:
            g.rgba = rgba

    sheet_top = pz - 0.0027
    desk_top = sheet_top - 0.001
    cx = px - 0.042
    box("floor", [2, 2, 0.05], [0, 0, 0], material="grid", gtype=mujoco.mjtGeom.mjGEOM_PLANE)
    box("desk", [0.22, 0.32, 0.02], [cx, 0, desk_top - 0.02], rgba=[0.55, 0.4, 0.28, 1])
    leg_h = (desk_top - 0.04) / 2
    for i, (dx, dy) in enumerate([(-0.2, 0.29), (-0.2, -0.29), (0.2, 0.29), (0.2, -0.29)]):
        box(f"desk_leg{i}", [0.02, 0.02, leg_h], [cx + dx, dy, leg_h], rgba=[0.15, 0.15, 0.17, 1])
    box("sheet", [0.105, 0.148, 0.0005], [px - 0.062, 0, sheet_top - 0.0005], rgba=[0.97, 0.97, 0.95, 1])
    return spec.compile()


# --------------------------------------------------------------------------- #
# Kinematics / dynamics from the controller's model
# --------------------------------------------------------------------------- #
class Arm:
    """Computes the operational-space quantities used by the controller.

    Uses its own MjData so it never touches the simulated robot's state.
    """

    def __init__(self, model: mujoco.MjModel, site="pen"):
        self.m = model
        self.d = mujoco.MjData(model)
        self.site = model.site(site).id
        self.body = model.site_bodyid[self.site]
        self.nv = model.nv
        self._J = np.zeros((3, self.nv))
        self._Jd = np.zeros((3, self.nv))
        self._M = np.zeros((self.nv, self.nv))

    # ---- light-weight evaluation used for gradients (no velocity terms) ----
    def J_M(self, q):
        d = self.d
        d.qpos[:] = q
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)
        mujoco.mj_crb(self.m, d)
        J = np.zeros((3, self.nv))
        mujoco.mj_jacSite(self.m, d, J, None, self.site)
        M = np.zeros((self.nv, self.nv))
        mujoco.mj_fullM(self.m, d, M)
        return J, M

    def Lambda(self, q):
        J, M = self.J_M(q)
        return np.linalg.inv(J @ np.linalg.solve(M, J.T))

    # ---- full evaluation at the current state ----
    def update(self, q, qd):
        m, d = self.m, self.d
        d.qpos[:] = q
        d.qvel[:] = qd
        mujoco.mj_forward(m, d)  # positions, M, bias (Coriolis + gravity)
        mujoco.mj_jacSite(m, d, self._J, None, self.site)
        mujoco.mj_jacDot(m, d, self._Jd, None, d.site_xpos[self.site], self.body)
        mujoco.mj_fullM(m, d, self._M)

        J, M = self._J.copy(), self._M.copy()
        Minv = np.linalg.inv(M)
        Lam = np.linalg.inv(J @ Minv @ J.T)  # apparent inertia, eq. (2)
        Jbar = Minv @ J.T @ Lam  # dynamically consistent inverse, eq. (3)
        self.x = d.site_xpos[self.site].copy()
        self.xd = J @ qd
        self.J, self.M, self.Minv, self.Lam, self.Jbar = J, M, Minv, Lam, Jbar
        self.Jdot_qd = self._Jd @ qd
        self.h = d.qfrc_bias.copy()  # C(q,q')q' + g(q)
        self.qfrc_passive = d.qfrc_passive.copy()  # -tau_f (joint viscous friction)
        self.N = np.eye(self.nv) - J.T @ Jbar.T  # null-space projector, eq. (10)
        return self


# --------------------------------------------------------------------------- #
# External torque / force estimation
# --------------------------------------------------------------------------- #
class MomentumObserver:
    """Generalized-momentum residual (De Luca et al. 2006), eq. (6)-(8).

        r = K_I [ p(t) - p(0) - int_0^t (tau_c + M'q' - h(q,q') + r) ds ]

    Since p' = tau_c + tau_ext - tau_f - h + M'q', the residual obeys
        r' = K_I (tau_ext - tau_f - r)
    i.e. r is a first-order filtered estimate of the external (+ unmodeled friction)
    joint torque. M' is obtained by finite differences of M.
    """

    def __init__(self, nv, K_I=100.0, dt=1e-3):
        self.K = K_I * np.ones(nv) if np.isscalar(K_I) else np.asarray(K_I)
        self.dt = dt
        self.r = np.zeros(nv)
        self.integral = np.zeros(nv)
        self.p0 = None

    def update(self, tau_prev, qd, M, h):
        """Call once per control step with the torque applied over the previous step and
        the current q', M(q), h(q,q'). Discretized to match MuJoCo's semi-implicit Euler
        step exactly, so r contains only external + friction torques."""
        if self.p0 is None:
            self.p0 = M @ qd
            self.M_prev, self.h_prev = M.copy(), h.copy()
            return self.r
        Mdot_qd = (M - self.M_prev) @ qd / self.dt
        self.integral += (tau_prev + Mdot_qd - self.h_prev + self.r) * self.dt
        self.r = self.K * (M @ qd - self.p0 - self.integral)
        self.M_prev, self.h_prev = M.copy(), h.copy()
        return self.r


# --------------------------------------------------------------------------- #
# Impedance parameter laws (Sec. VI)
# --------------------------------------------------------------------------- #
def damping_law(xdot, a=60.0, b=4.0, D_min=5.0):
    """Eq. (20), per Cartesian direction. (The paper prints 'min'; the text makes clear
    that 5 N s/m is a lower saturation, i.e. max.)"""
    return np.maximum(a * np.exp(-b * np.abs(xdot)), D_min)


def time_constant_L4(D, lam_f=3.0, D_f=30.0, a=1.1820, b=0.60, c=0.4, d=20.0):
    """Minimum-T curve of the stability region (law L4)."""
    return lam_f / ((a + b * np.arctan(c * (D - d))) * D_f)


T_L3 = 0.764 / 5.0  # constant T that stays right of the lambda_min = 0.764 kg frontier (Fig. 3) for D >= 5


@dataclass
class ImpedanceLaw:
    """Desired end-effector inertia/damping, per Cartesian direction.

    kind:
      'const' : constant lam, D  (lam may be a scalar or a 3-vector)
      'var'   : variable D from eq. (20), constant lam (scalar or 3-vector)
      'L1'    : variable D, lam = 1.1 kg          (Sec. VI)
      'L2'    : variable D, lam = 5.6 kg
      'L3'    : variable D, constant time constant T = lam/D
      'L4'    : variable D, minimum time constant T(D) of the stability region
    """
    kind: str = "L1"
    lam: object = 1.1
    D: float = 60.0
    a: float = 60.0
    b: float = 4.0
    D_min: float = 5.0
    name: str = ""

    def __call__(self, xdot):
        """Returns diagonal (lambda_d, D_d) as 3-vectors."""
        if self.kind == "const":
            return np.broadcast_to(np.asarray(self.lam, float), (3,)).copy(), np.full(3, float(self.D))
        D = damping_law(xdot, self.a, self.b, self.D_min)
        if self.kind == "var":
            lam = np.broadcast_to(np.asarray(self.lam, float), (3,)).copy()
        elif self.kind == "L1":
            lam = np.full(3, 1.1)
        elif self.kind == "L2":
            lam = np.full(3, 5.6)
        elif self.kind == "L3":
            lam = D * T_L3
        elif self.kind == "L4":
            lam = D * time_constant_L4(D)
        else:
            raise ValueError(self.kind)
        return lam, D

    @property
    def label(self):
        if self.name:
            return self.name
        if self.kind == "const":
            return f"const (λ={np.round(self.lam, 3)} kg, D={self.D:g} Ns/m)"
        if self.kind == "var":
            return f"variable D, λ={np.round(self.lam, 3)} kg"
        return {"L1": "variable D, λ=1.1 kg (L1)", "L2": "variable D, λ=5.6 kg (L2)",
                "L3": "variable D, const T (L3)", "L4": "variable D, min T (L4)"}[self.kind]


# --------------------------------------------------------------------------- #
# Redundancy resolution (Sec. IV)
# --------------------------------------------------------------------------- #
def manipulability(J):
    """Eq. (11): m(q) = det(J J^T) (translational Jacobian)."""
    return np.linalg.det(J @ J.T)


def dci(Lam, mu=10.0):
    """Eq. (14)-(17): omega = 1/2 E^T W E, E = [diag(Lam) - sigma, offdiag(Lam)]."""
    sigma = np.trace(Lam) / 3.0
    E = np.array([Lam[0, 0] - sigma, Lam[1, 1] - sigma, Lam[2, 2] - sigma,
                  Lam[0, 1], Lam[0, 2], Lam[1, 2]])
    W = np.array([1, 1, 1, mu, mu, mu], dtype=float)
    return 0.5 * E @ (W * E)


class Redundancy:
    """Secondary task torque u in eq. (10). kind: 'none' | 'manip' | 'dci'."""

    def __init__(self, arm: Arm, kind="dci", k_m=20.0, k_c=1.0, mu=10.0, eps=1e-4, every=5):
        self.arm, self.kind = arm, kind
        self.k_m, self.k_c, self.mu, self.eps, self.every = k_m, k_c, mu, eps, every
        self.u = np.zeros(arm.nv)
        self.count = 0

    def _index(self, q):
        if self.kind == "manip":
            J, _ = self.arm.J_M(q)
            return manipulability(J)
        return dci(self.arm.Lambda(q), self.mu)

    def gradient(self, q):
        g = np.zeros_like(q)
        for i in range(len(q)):
            dq = np.zeros_like(q)
            dq[i] = self.eps
            g[i] = (self._index(q + dq) - self._index(q - dq)) / (2 * self.eps)
        return g

    def __call__(self, q):
        if self.kind == "none":
            return self.u
        if self.count % self.every == 0:  # gradient refreshed at 200 Hz
            g = self.gradient(q)
            self.u = self.k_m * g if self.kind == "manip" else -self.k_c * g  # eq. (12) / (18)
        self.count += 1
        return self.u


# --------------------------------------------------------------------------- #
# The controller, eq. (9) + (10)
# --------------------------------------------------------------------------- #
class VariableImpedanceController:
    def __init__(self, model, law: ImpedanceLaw, redundancy="dci", K_I=100.0, k_D=5.0,
                 force_source="observer", **red_kw):
        self.arm = Arm(model)
        self.law = law
        self.red = Redundancy(self.arm, redundancy, **red_kw)
        self.obs = MomentumObserver(model.nv, K_I, model.opt.timestep)
        self.k_D = k_D
        self.force_source = force_source
        self.tau = np.zeros(model.nv)
        self.F_hat = np.zeros(3)

    def __call__(self, q, qd, F_true=None):
        a = self.arm.update(q, qd)

        # external force: momentum observer (default) or ideal F/T sensor
        r = self.obs.update(self.tau, qd, a.M, a.h)
        self.F_hat = a.Jbar.T @ r  # eq. (8)
        F = self.F_hat if self.force_source == "observer" or F_true is None else F_true

        lam_d, D_d = self.law(a.xd)
        self.lam_d, self.D_d = lam_d, D_d

        # eq. (9):  tau_imp = -J^T Lam [ J'q' + Lam_d^-1 (D_d x' - F) ] + g - r
        # The paper lumps C q' into r; here r -> tau_ext - tau_f only, so C q' + g = h is
        # compensated with the model (equivalent once the observer has converged).
        if self.force_source == "observer" or F_true is None:
            comp = a.h - r
        else:  # ideal F/T sensor + known friction: tau = J^T F_c + h + tau_f, F_c from eq. (5)
            comp = a.h - a.qfrc_passive - a.J.T @ F
        tau_imp = -a.J.T @ a.Lam @ (a.Jdot_qd + (D_d * a.xd - F) / lam_d) + comp

        # eq. (10): null-space secondary task + joint damping. The damping is weighted by
        # each joint's own inertia (k_D in 1/s), so every joint gets the same damping rate:
        # with a scalar gain [N m s/rad] the light iiwa wrist (M77 ~ 0.001 kg m^2) would be
        # over-damped beyond what a 1 kHz loop with 2 ms latency can realize.
        u = self.red(q)
        tau = tau_imp + a.N @ (u - self.k_D * np.diag(a.M) * qd)
        self.tau = tau
        return tau
