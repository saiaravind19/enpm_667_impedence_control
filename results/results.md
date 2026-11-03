## A1 Redundancy vs stability (low virtual inertia, variable damping)

Low inertia Λd,low = 0.27·diag(Λ(q_i)) = [0.106, 0.135, 1.283] kg (same ratio as the paper's diag{0.0328, 0.0548, 0.8138}).

| impedance | null-space task | runs | unstable | H [s] | e [mm] | mean path dev. [mm] | mean |f| [N] | mean DCI |
|---|---|---|---|---|---|---|---|---|
| var D, Λd = 1×low | manip | 1 | 1 | nan ± nan | 246.2 ± 0.0 | 1.25 ± 0.00 | 2.73 | 10.10 |
| var D, Λd = 1×low | dci | 1 | 0 | 15.07 ± 0.00 | 17.8 ± 0.0 | 1.13 ± 0.00 | 2.36 | 6.40 |
| var D, Λd = 1.5×low | manip | 1 | 1 | nan ± nan | 224.5 ± 0.0 | 1.23 ± 0.00 | 2.78 | 10.98 |
| var D, Λd = 1.5×low | dci | 1 | 0 | 15.07 ± 0.00 | 17.9 ± 0.0 | 1.13 ± 0.00 | 2.36 | 6.40 |
| var D, Λd = 2×low | manip | 1 | 1 | nan ± nan | 203.9 ± 0.0 | 1.30 ± 0.00 | 2.77 | 12.13 |
| var D, Λd = 2×low | dci | 1 | 0 | 15.07 ± 0.00 | 18.1 ± 0.0 | 1.13 ± 0.00 | 2.36 | 6.40 |
| var D, Λd = 3×low | manip | 1 | 0 | 15.98 ± 0.00 | 13.8 ± 0.0 | 1.27 ± 0.00 | 2.73 | 22.16 |
| var D, Λd = 3×low | dci | 1 | 0 | 15.07 ± 0.00 | 18.6 ± 0.0 | 1.13 ± 0.00 | 2.36 | 6.40 |


## A2-A4 Writing task, 5 simulated subjects (mean ± std)

| impedance | null-space task | runs | unstable | H [s] | e [mm] | mean path dev. [mm] | mean |f| [N] | mean DCI |
|---|---|---|---|---|---|---|---|---|
| const high D (λ=1.1, D=60) | none | 5 | 0 | nan ± nan | 177.2 ± 7.5 | 15.17 ± 0.22 | 7.47 | 28.00 |
| variable D, λ=1.1 kg (L1) | none | 5 | 0 | nan ± nan | 163.1 ± 7.3 | 15.31 ± 0.21 | 7.47 | 28.14 |
| const high D (λ=1.1, D=60) | manip | 5 | 0 | 17.05 ± 0.73 | 11.5 ± 4.0 | 1.39 ± 0.24 | 2.98 | 23.01 |
| variable D, λ=1.1 kg (L1) | manip | 5 | 0 | 16.10 ± 0.78 | 15.1 ± 2.6 | 1.40 ± 0.24 | 2.71 | 22.15 |
| const high D (λ=1.1, D=60) | dci | 5 | 0 | 16.11 ± 0.75 | 19.0 ± 2.9 | 1.32 ± 0.14 | 2.71 | 6.39 |
| variable D, λ=1.1 kg (L1) | dci | 5 | 0 | 15.17 ± 0.52 | 28.1 ± 5.0 | 1.28 ± 0.13 | 2.37 | 6.40 |
| variable D, λ=5.6 kg (L2) | dci | 5 | 0 | 15.15 ± 0.54 | 32.9 ± 5.6 | 1.34 ± 0.14 | 2.36 | 6.40 |
| variable D, const T (L3) | dci | 5 | 0 | 15.15 ± 0.54 | 33.9 ± 5.9 | 1.38 ± 0.14 | 2.36 | 6.40 |
| variable D, min T (L4) | dci | 5 | 0 | 15.15 ± 0.54 | 30.7 ± 5.1 | 1.30 ± 0.13 | 2.37 | 6.40 |
| const low D (λ=1.1, D=20) | dci | 5 | 0 | 13.30 ± 1.01 | 124.4 ± 16.4 | 2.07 ± 0.06 | 1.47 | 6.41 |

Null-space task 'none' (damping only): the self-motion drives joint 6 into its +120 deg limit after ~4.5 s and the drawing is never completed (H = nan), so its e and path deviation describe a failed run, not accuracy.


## A4 Paired t-test, variable vs constant impedance (5 subjects; h=1 ↔ p<0.05)

| metric | var vs hconst | var vs lconst | hconst vs lconst |
|---|---|---|---|
| time H | var<hconst, p = 0.00166 (h=1) | var>lconst, p = 0.00417 (h=1) | hconst>lconst, p = 0.00047 (h=1) |
| length error e | var>hconst, p = 0.046 (h=1) | var<lconst, p = 0.00077 (h=1) | hconst<lconst, p = 0.000168 (h=1) |
| mean path deviation | var<hconst, p = 0.134 (h=0) | var<lconst, p = 8.46e-05 (h=1) | hconst<lconst, p = 0.00013 (h=1) |
| mean force | var<hconst, p = 0.000183 (h=1) | var>lconst, p = 7.89e-09 (h=1) | hconst>lconst, p = 1.33e-06 (h=1) |


## A5 Observer
RMS error of F_hat vs true human force (L1, DCI, 5 subjects): 0.699 N (mean |f| = 2.37 N)

