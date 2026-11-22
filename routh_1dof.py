"""
1-DOF linear stability check used in Sec. 4.6 of the report (my own analysis, not in the paper).

Plant: one Cartesian direction with physical apparent mass m.
Controller (eq. 5 with force estimate):  F_c = -(m/lam) D x' + (m/lam - 1) F_hat
Observer: F_hat = K/(s+K) F           (first-order momentum observer, eq. 6-7)
Latency:  e^{-s tau} on F_c          (1st-order Pade approximation)
Human:    F = -k x                    (hand modelled as a spring around a fixed point)

With alpha = lam/m the closed-loop characteristic polynomial is built below and its
roots are checked. Prints the smallest stable alpha for each (k, D).

    python routh_1dof.py
"""
import numpy as np
from numpy.polynomial import polynomial as P


def char_poly(alpha, D, k, m=4.77, K=100.0, tau=0.002):
    sK = [K, 1.0]                 # (s + K)
    den = [1.0, tau / 2]          # Pade denominator (1 + s tau/2)
    num = [1.0, -tau / 2]         # Pade numerator   (1 - s tau/2)
    lhs = P.polymul(P.polymul([0, 0, m], sK), den)              # m s^2 (s+K)(1+s tau/2)
    damp = P.polymul(P.polymul(num, [0, D / alpha]), sK)        # (1-s tau/2)(D/alpha) s (s+K)
    ffb = P.polymul(num, [k * (1 / alpha - 1) * K])             # (1-s tau/2)(1/alpha-1) K k
    hand = P.polymul(P.polymul([k], sK), den)                   # k (s+K)(1+s tau/2)
    return P.polyadd(P.polyadd(P.polyadd(lhs, damp), ffb), hand)


def stable(alpha, D, k, **kw):
    c = char_poly(alpha, D, k, **kw)
    return bool((np.roots(c[::-1]).real < 0).all())


if __name__ == "__main__":
    alphas = np.linspace(0.01, 1.0, 991)
    for k in [200.0, 1000.0, 2000.0]:
        row = []
        for D in [5, 10, 20, 30, 40, 50, 60]:
            s = np.array([stable(a, D, k) for a in alphas])
            row.append(f"D={D}: alpha_min={alphas[s].min():.2f}" if s.any() else f"D={D}: none")
        print(f"k = {k:6.0f} N/m  ", "  ".join(row))
