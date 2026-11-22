"""
Cooperative writing task (paper Sec. VII): a human guides the pen along a path on a
horizontal sheet that has straight segments, sharp corners and a circular arc.

Human model (paper Sec. VI, Fig. 5): the arm is a pure stiffness k = 200 N/m pulling the
pen toward the point the human intends to reach,
        f = k (x_h - x).
Here x_h is a "carrot" moving along the drawn path. The human slows down near corners
and on the arc, and waits for the pen if it lags too far behind (|x_h - x| > lag_max).
That makes the execution time depend on how easily the robot follows, like a real
operator who must finish the drawing accurately.
"""

from __future__ import annotations

import numpy as np


class WritingPath:
    """Polyline + arc path on the plane z = z0, starting at the pen's start point."""

    def __init__(self, start_xy, z0, ds=0.0005):
        x0, y0 = start_xy
        pts, kinds = [], []

        def line(p, q):
            n = max(2, int(np.linalg.norm(np.subtract(q, p)) / ds))
            t = np.linspace(0, 1, n, endpoint=False)
            return [(p[0] + (q[0] - p[0]) * ti, p[1] + (q[1] - p[1]) * ti) for ti in t]

        def arc(c, r, th0, th1):
            n = max(2, int(abs(th1 - th0) * r / ds))
            th = np.linspace(th0, th1, n, endpoint=False)
            return [(c[0] + r * np.cos(t), c[1] + r * np.sin(t)) for t in th]

        P0 = (x0, y0)
        P1 = (x0, y0 + 0.10)
        P2 = (x0 - 0.12, y0 + 0.10)
        C = (x0 - 0.12, y0 + 0.05)
        P3 = (x0 - 0.12, y0)
        P4 = (x0, y0)
        P5 = (x0, y0 - 0.10)
        P6 = (x0 - 0.12, y0 - 0.10)

        segs = [("line", line(P0, P1)), ("line", line(P1, P2)),
                ("arc", arc(C, 0.05, np.pi / 2, 3 * np.pi / 2)),
                ("line", line(P3, P4)), ("line", line(P4, P5)), ("line", line(P5, P6))]
        for kind, s in segs:
            kinds += [kind] * len(s)
            pts += s
        pts.append(P6)
        kinds.append("line")

        xy = np.array(pts)
        self.xyz = np.column_stack([xy, np.full(len(xy), z0)])
        seglen = np.linalg.norm(np.diff(self.xyz, axis=0), axis=1)
        self.s = np.concatenate([[0], np.cumsum(seglen)])
        self.length = self.s[-1]
        self.kinds = np.array(kinds)
        self.z0 = z0
        # sharp corners at P1, P4, P5 (the line P1-P2 meets the arc tangentially, so P2 is smooth)
        self.corners = np.array([self.s_of(P1), self.s_of(P4), self.s_of(P5)])
        self.corner_pts = np.array([P1, P4, P5])

    def s_of(self, p):
        """Arc length of the path point closest to p, searched in order (first hit)."""
        d = np.linalg.norm(self.xyz[:, :2] - np.asarray(p), axis=1)
        idx = np.where(d < 1e-6 + d.min())[0]
        # P4 coincides with P0: take the later occurrence for P4
        return self.s[idx[-1]] if len(idx) > 1 else self.s[idx[0]]

    def point(self, s):
        s = np.clip(s, 0, self.length)
        return np.array([np.interp(s, self.s, self.xyz[:, i]) for i in range(3)])

    def speed(self, s, v_line=0.10, v_arc=0.06, v_corner=0.02, r_slow=0.03, v_end=0.02):
        """Nominal speed the human wants at arc length s."""
        i = min(np.searchsorted(self.s, s), len(self.s) - 1)
        v = v_arc if self.kinds[i] == "arc" else v_line
        dc = np.min(np.abs(self.corners - s))
        v = min(v, v_corner + (v - v_corner) * min(1.0, dc / r_slow))
        # start-up and stopping
        v = min(v, v_end + (v_line - v_end) * min(1.0, s / r_slow),
                v_end + (v_line - v_end) * min(1.0, (self.length - s) / r_slow))
        return v

    def distance_to_path(self, p):
        """Distance from point(s) p (N,3) to the polyline (xy only)."""
        p = np.atleast_2d(p)[:, :2]
        A, B = self.xyz[:-1, :2], self.xyz[1:, :2]
        AB = B - A
        L2 = np.maximum((AB**2).sum(1), 1e-12)
        out = np.empty(len(p))
        for k, pk in enumerate(p):
            t = np.clip(((pk - A) * AB).sum(1) / L2, 0, 1)
            proj = A + t[:, None] * AB
            out[k] = np.sqrt(((proj - pk) ** 2).sum(1)).min()
        return out


class HumanOperator:
    """Spring-like human arm pulling the pen toward a carrot that moves along the path."""

    def __init__(self, path: WritingPath, k=200.0, k_z=400.0, lag_max=0.04,
                 noise_std=0.8, noise_bw=4.0, tremor_amp=0.3, tremor_hz=9.0, seed=0,
                 speed_scale=1.0):
        """noise_*: band-limited force fluctuation of the hand (1st-order filtered white
        noise, std [N], bandwidth [Hz]); tremor_*: physiological tremor (8-12 Hz).
        seed selects a 'subject'; speed_scale scales the subject's preferred speed."""
        self.path, self.k, self.k_z, self.lag_max = path, k, k_z, lag_max
        self.noise_std, self.noise_bw = noise_std, noise_bw
        self.tremor_amp, self.tremor_hz = tremor_amp, tremor_hz
        self.rng = np.random.default_rng(seed)
        self.phase = self.rng.uniform(0, 2 * np.pi, 2)
        self.speed_scale = speed_scale
        self.n = np.zeros(2)
        self.s = 0.0

    def target(self):
        return self.path.point(self.s)

    def force(self, x, t, dt):
        xh = self.target()
        lag = np.linalg.norm((xh - x)[:2])
        gate = np.clip(1.0 - lag / self.lag_max, 0.0, 1.0)
        self.s = min(self.path.length, self.s + self.speed_scale * self.path.speed(self.s) * gate * dt)
        xh = self.target()
        e = xh - x
        f = np.array([self.k * e[0], self.k * e[1], self.k_z * e[2]])
        # hand force fluctuations: OU process with the requested std and bandwidth
        if self.noise_std > 0:
            a = 2 * np.pi * self.noise_bw
            self.n += -a * self.n * dt + self.noise_std * np.sqrt(2 * a * dt) * self.rng.standard_normal(2)
            f[:2] += self.n
        if self.tremor_amp > 0:
            f[:2] += self.tremor_amp * np.sin(2 * np.pi * self.tremor_hz * t + self.phase)
        return f, xh
