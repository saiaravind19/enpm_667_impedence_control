"""
Renders the simulation videos used in the report/presentation to results/videos/.

    python render_videos.py            # all clips
    python render_videos.py --only main

Clips
  main        : variable impedance L1 + DCI null-space task, full writing task
  hconst      : constant high damping (D = 60)
  manip_low   : low virtual inertia + manipulability null-space task (goes unstable)
  dci_low     : low virtual inertia + DCI null-space task (stays stable)

Black line = path the human wants to draw, red = pen trace, green ball = point the
human is pulling toward.
"""

import argparse
import os

import mujoco
import numpy as np

from vic import Q_INIT, load_model, Arm, ImpedanceLaw
from writing_sim import run

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results", "videos")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    dq = np.diag(Arm(load_model()).Lambda(Q_INIT))
    low = ImpedanceLaw("var", lam=0.27 * dq, name="variable D, low inertia")
    clips = {
        "main": (ImpedanceLaw("L1"), "dci"),
        "hconst": (ImpedanceLaw("const", lam=1.1, D=60.0), "dci"),
        "manip_low": (low, "manip"),
        "dci_low": (low, "dci"),
    }
    for name, (law, red) in clips.items():
        if a.only and name != a.only:
            continue
        path = os.path.join(OUT, f"{name}.mp4")
        print("rendering", path)
        run(law, red, video=path, T_max=25.0)


if __name__ == "__main__":
    main()
