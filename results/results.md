## A1 Redundancy vs stability (low virtual inertia, variable damping)

Low inertia Λd,low = 0.27·diag(Λ(q_i)) = [0.073, 0.087, 0.646] kg (same ratio as the paper's diag{0.0328, 0.0548, 0.8138}).

| impedance | null-space task | runs | unstable | H [s] | e [mm] | mean path dev. [mm] | mean |f| [N] | mean DCI |
|---|---|---|---|---|---|---|---|---|
| var D, Λd = 1×low | manip | 1 | 1 | nan ± nan | 322.3 ± 0.0 | 1.47 ± 0.00 | 2.44 | 3.94 |
| var D, Λd = 1×low | dci | 1 | 0 | 15.07 ± 0.00 | 15.4 ± 0.0 | 1.08 ± 0.00 | 2.39 | 1.55 |
| var D, Λd = 1.5×low | manip | 1 | 1 | nan ± nan | 245.1 ± 0.0 | 1.43 ± 0.00 | 2.57 | 4.49 |
| var D, Λd = 1.5×low | dci | 1 | 0 | 15.07 ± 0.00 | 15.8 ± 0.0 | 1.08 ± 0.00 | 2.39 | 1.55 |
| var D, Λd = 2×low | manip | 1 | 1 | nan ± nan | 186.6 ± 0.0 | 1.46 ± 0.00 | 2.50 | 5.53 |
| var D, Λd = 2×low | dci | 1 | 0 | 15.07 ± 0.00 | 16.3 ± 0.0 | 1.08 ± 0.00 | 2.38 | 1.55 |
| var D, Λd = 3×low | manip | 1 | 0 | 15.41 ± 0.00 | 14.4 ± 0.0 | 1.33 ± 0.00 | 2.50 | 8.09 |
| var D, Λd = 3×low | dci | 1 | 0 | 15.07 ± 0.00 | 17.5 ± 0.0 | 1.08 ± 0.00 | 2.38 | 1.55 |


## A2-A4 Writing task, 5 simulated subjects (mean ± std)

| impedance | null-space task | runs | unstable | H [s] | e [mm] | mean path dev. [mm] | mean |f| [N] | mean DCI |
|---|---|---|---|---|---|---|---|---|
| const high D (λ=1.1, D=60) | none | 5 | 0 | 16.61 ± 0.73 | 25.7 ± 3.8 | 1.34 ± 0.18 | 2.83 | 2.30 |
| variable D, λ=1.1 kg (L1) | none | 5 | 0 | 15.54 ± 0.71 | 29.3 ± 3.8 | 1.28 ± 0.17 | 2.53 | 2.31 |
| const high D (λ=1.1, D=60) | manip | 5 | 0 | 16.55 ± 0.78 | 20.0 ± 2.8 | 1.42 ± 0.14 | 2.79 | 8.53 |
| variable D, λ=1.1 kg (L1) | manip | 5 | 0 | 15.50 ± 0.74 | 23.9 ± 2.2 | 1.38 ± 0.15 | 2.49 | 8.11 |
| const high D (λ=1.1, D=60) | dci | 5 | 0 | 16.20 ± 0.76 | 29.9 ± 3.7 | 1.36 ± 0.14 | 2.71 | 1.55 |
| variable D, λ=1.1 kg (L1) | dci | 5 | 0 | 15.14 ± 0.56 | 36.4 ± 5.1 | 1.31 ± 0.17 | 2.37 | 1.55 |
| variable D, λ=5.6 kg (L2) | dci | 5 | 0 | 15.16 ± 0.50 | 57.9 ± 7.7 | 1.57 ± 0.18 | 2.33 | 1.55 |
| variable D, const T (L3) | dci | 5 | 0 | 15.17 ± 0.50 | 60.8 ± 8.4 | 1.72 ± 0.19 | 2.32 | 1.55 |
| variable D, min T (L4) | dci | 5 | 0 | 15.14 ± 0.55 | 47.6 ± 5.7 | 1.40 ± 0.18 | 2.37 | 1.55 |
| const low D (λ=1.1, D=20) | dci | 5 | 0 | 12.99 ± 0.94 | 103.5 ± 13.1 | 1.99 ± 0.14 | 1.48 | 1.56 |


## A4 Paired t-test, variable vs constant impedance (5 subjects; h=1 ↔ p<0.05)

| metric | var vs hconst | var vs lconst | hconst vs lconst |
|---|---|---|---|
| time H | var<hconst, p = 0.00117 (h=1) | var>lconst, p = 0.000365 (h=1) | hconst>lconst, p = 1.22e-05 (h=1) |
| length error e | var>hconst, p = 0.159 (h=0) | var<lconst, p = 0.000454 (h=1) | hconst<lconst, p = 0.000229 (h=1) |
| mean path deviation | var<hconst, p = 0.122 (h=0) | var<lconst, p = 0.000249 (h=1) | hconst<lconst, p = 0.000512 (h=1) |
| mean force | var<hconst, p = 0.000154 (h=1) | var>lconst, p = 4.43e-09 (h=1) | hconst>lconst, p = 1.13e-06 (h=1) |


## A5 Observer
RMS error of F_hat vs true human force (L1, DCI, 5 subjects): 0.677 N (mean |f| = 2.37 N)

