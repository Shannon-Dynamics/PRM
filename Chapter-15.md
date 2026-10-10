# Chapter 15 — Kalman Filtering: The Geometric Observer

> Part IV — Planning with Uncertainty · Estimated length: 7 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Fourteen chapters of planners assumed the planner knew $q$. This chapter admits that Rusty does
not: encoders drift, landmarks are noisy, and the configuration a planner imagines is only a
*belief* about the state a robot occupies. Choset's Chapter 8 teaches the Kalman filter as no other
robotics text does — not as Bayes rule with Gaussians (the sister book does that, completely) but
as an **observer**: a copy of the system corrected toward the hyperplane of states consistent with
the latest measurement. The "aha" is geometric and single: **the Kalman filter is a Luenberger
observer whose gain is re-chosen every step so that the correction is the shortest step, in
Mahalanobis distance, onto the most likely measurement hyperplane.** Once the reader sees the
ellipse touch the hyperplane (Figs. 8.3–8.4), the gain $R$, the innovation covariance $S$, $\chi^2$
association, and Kalman SLAM fall out of one picture — and so does the warning that an optimal
observer can still never learn a direction the sensors do not see (observability). Everything
deeper — completing the square, UKF, manifolds, EKF-SLAM inconsistency, factor graphs — is a
cross-link to the sister volume. Compressed by decision, the chapter gives a *planner* exactly the
estimation vocabulary Chapters 16 and 23 consume.

**Story line**
1. **Hook** — Rusty dead-reckons a loop of the Apartment; the orange executed trajectory peels away
   from the gray-dashed truth (Fig. 8.1's drift on our simulator). One landmark sighting snaps it back.
2. **Play (C)** — w15.1 Observer Bench: a fixed-gain observer on Rusty's 1D dead reckoning; turn
   the gain knob; press "let the filter choose" and the knob moves by itself — the gain schedule.
3. **Formalize (F)** — the simple observer as projection onto $\Omega$; adding $P$ and the
   Mahalanobis inner product; adding sensor noise and the most likely output $y^*$; the Kalman
   summary (§8.2.5); two hand-checked dead-reckoning steps; observability by rank (Thm. 8.2.2).
4. **Algorithm** — the EKF (§8.3) for range-and-bearing and range-only localization; data
   association by minimum Mahalanobis innovation (§8.3.2); Kalman SLAM (§8.4).
5. **Implementation (P)** — the thin `estimate` crate: Luenberger `Observer`, observability test,
   Choset-notation adapters over the sister's ported `Kf`/`Ekf`, landmark models, $\chi^2$
   association, a two-landmark SLAM instance.
6. **Integration lab** — Rusty localizes against the Apartment's door-corner landmarks with one
   visible (w15.2: the ellipse that never shrinks), then two; then SLAM rebuilds forgotten landmarks.
7. **Close** — the planner owns a Gaussian $bel(x_t)$ with a covariance it can read. Chapter 16
   asks what to do when the belief is not Gaussian, and how to plan on it.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (Rusty, the Apartment, the range sensor, the `sim`/`geom`/`prob` ports — the
  filter's $x_t$ *is* the planner's $q \in SE(2)$ for Rusty, stated once here), Ch. 5 ($SE(2)$ as a
  manifold; why heading needs care). Appendix E carries App. J.1–J.3; this chapter uses J.1, J.4, J.5.
- **Feeds into:** Ch. 16 (the Gaussian as one of three representations; association as why beliefs
  go multimodal), Ch. 19 (MPC consumes $bel(x_t)$), Ch. 23 (belief panel imports `estimate`).
- **Baseline sources:** C §8.1 (b. 270–272), §8.2.1–8.2.7 (b. 272–289: Thms. 8.2.1–8.2.2, the
  §8.2.6 example), §8.3 (b. 289–294: eqs. 8.37–8.45, Jacobians, data association, range-only),
  §8.4.1–8.4.2 (b. 294–297); App. J.1, J.4 (eq. J.9, Thm. J.4.1), J.5 ($F = I + TA$, Thms.
  J.5.1–J.5.2). App. I is **not** used: the sister book's Ch. 2 is the Gaussian primer —
  `https://shannon-dynamics.github.io/ProbabilisticRoboticRust/chapters/ch02-probability`. Sister
  cross-links (same prefix): `ch05-bayes-filter` (the recursion the KF instantiates),
  `ch06-kalman-filters` (completing the square, information form, RTS), `ch07-ekf-ukf-manifolds`
  (UKF, error-state EKF on $SE(2)$), `ch11-localization-gaussian` (gating, MHT), `ch14-ekf-slam`
  (quadratic cost, inconsistency), `ch15-factor-graphs` (what replaced Kalman SLAM). Modernization
  set: Barfoot, *State Estimation for Robotics* (verify); Huang, Mourikis & Roumeliotis,
  observability-based consistency of EKF-SLAM (verify).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (Choset's bracket notation is kept because the geometry reads better in it;
the sister book's symbols sit beside each so nobody translates twice):

| Symbol | Meaning | Sister book |
|---|---|---|
| $x(k) \in \mathbb{R}^n$, $u(k) \in \mathbb{R}^m$, $y(k) \in \mathbb{R}^p$ | state, input, output at sample $k$ (period $T$) | $x_t, u_t, z_t$ |
| $F(k), G(k), H(k)$ | dynamics, input, output matrices | $A_t, B_t, C_t$ |
| $v(k) \sim \mathcal{N}(0, V)$, $w(k) \sim \mathcal{N}(0, W)$ | process and measurement noise | $\mathcal{N}(0, R_t)$, $\mathcal{N}(0, Q_t)$ |
| $\hat x(k_1 \mid k_2)$, $P(k_1 \mid k_2)$ | estimate and error covariance at $k_1$ given outputs through $k_2$ | $\mu_t, \Sigma_t$ ($\bar\mu_t, \bar\Sigma_t$ predicted) |
| $\Omega = \{x : Hx = y\}$, $\Omega^*$ | states consistent with $y$; with the most likely output $y^*$ | — |
| $\nu = y - H\hat x(k{+}1 \mid k)$, $S = HPH^\top + W$ | innovation and its covariance | $z_t - C_t\bar\mu_t$, $S_t$ |
| $R = PH^\top S^{-1}$ | **Kalman gain** (Choset's letter; not a covariance) | $K_t$ |
| $\lVert x\rVert_M^2 = x^\top P^{-1} x$; $Q = [H;\, HF;\, \dots;\, HF^{n-1}]$ | Mahalanobis norm under $P(k{+}1 \mid k)$; observability matrix (Thm. 8.2.2) | same; — |
| $a(\cdot)$, $\chi^2_{ij} = \nu_{ij}^\top S_{ij}^{-1}\nu_{ij}$ | association map; Mahalanobis innovation of measurement $i$ vs. landmark $j$ | $c_t^i$, $d_{ij}$ |

The prose says once, precisely: *Choset's $R$ is the gain and $V, W$ are covariances; the sister
book (Thrun's convention) uses $K$ for the gain and $R, Q$ for covariances. Widgets label both.*
Estimation colors are imported unchanged: prior `--pr-prior` (blue), prediction `--pr-prediction`
(orange), measurement `--pr-measurement` (green), posterior `--pr-posterior` (purple), truth
`--pr-truth` (gray dashed).

**Definitions** (display boxes):
- *Observer*: a recursion reconstructing $x(k)$ from $u(\cdot), y(\cdot)$ and the model
  $(F, G, H)$ when $H$ is not invertible (§8.2.1; App. J.4).
- *Innovation* $\nu$: what the sensor said minus what it would have said had the prediction been
  exact. *Consistent set* $\Omega$: the affine subspace of states reproducing the output; dimension
  $n - p$ when $H$ has full row rank (assumed throughout §8.2).
- *Mahalanobis distance* $d_M(x, \hat x) = \lVert x - \hat x\rVert_M$: distance in standard
  deviations under the predicted covariance (§8.2.3, footnote 5).
- *Observable* (App. J.4; §8.2.7): the initial state is determined by $u, y$ over a finite window;
  for LTI systems iff $\operatorname{rank} Q = n$. *Luenberger observer* (eq. J.9):
  $\dot{\hat x} = A\hat x + Bu + K(y - C\hat x)$, error dynamics $\dot e = (A - KC)e$.
- *Data association*: choosing $a(i)$, the landmark that explains measurement $i$ (§8.3.2).

**Derivations / theorems**

1. **The simple observer is an orthogonal projection** (§8.2.2). *Statement:* with no noise, the
   update closest to the prediction is $\hat x^+ = \hat x^- + H^\top (HH^\top)^{-1}\nu$. *Sketch:*
   (i) the true state lies in $\Omega$; (ii) the shortest $\Delta x$ into $\Omega$ is orthogonal to
   it; (iii) vectors orthogonal to $\operatorname{null}(H)$ lie in $\operatorname{column}(H^\top)$,
   so $\Delta x = H^\top\gamma$; (iv) guess $\gamma = K\nu$, enforce $H\Delta x = \nu$;
   (v) $K = (HH^\top)^{-1}$. *Collapsible:* C problems 4–6; why components parallel to $\Omega$ are
   never corrected — the defect the covariance fixes.
2. **Prediction grows the ellipse; the update is the Mahalanobis-closest point** (§8.2.3).
   *Statement:* $P(k{+}1 \mid k) = FPF^\top + V$; with noiseless outputs
   $\Delta x = P H^\top (HPH^\top)^{-1}\nu$. *Sketch:* (i) expand $E[(x - \hat x)(x - \hat x)^\top]$
   after substituting the dynamics; (ii) cross terms vanish since $v(k)$ is zero-mean and
   independent of the error; (iii) replace Euclidean orthogonality by $\langle a, \Delta x\rangle_M
   = 0$ for $a \in \operatorname{null}(H)$; (iv) hence $\Delta x \in \operatorname{column}(PH^\top)$;
   (v) solve as before. *Collapsible:* C problem 7; why this observer drives $P$ singular.
3. **The Kalman filter: most likely output, then project** (§8.2.4, Thm. 8.2.1). *Statement:*
   with $w \sim \mathcal{N}(0, W)$: $R = P^-H^\top(HP^-H^\top + W)^{-1}$, $\hat x^+ = \hat x^- + R\nu$,
   $P^+ = P^- - RHP^-$. *Sketch:* (i) project the prediction into output space, $(\hat y, \hat W)
   = (H\hat x^-, HP^-H^\top)$; (ii) the measurement is a second Gaussian $(y, W)$ there;
   (iii) **Thm. 8.2.1**: the product of Gaussians $(z_1, C_1), (z_2, C_2)$ is proportional to one
   with $z_3 = z_1 + Q(z_2 - z_1)$, $C_3 = C_1 - QC_1$, $Q = C_1(C_1 + C_2)^{-1}$; (iv) its peak
   $y^*$ defines $\Omega^*$; (v) project onto $\Omega^*$ as in derivation 2; the two inverses
   collapse into $(HP^-H^\top + W)^{-1}$. *Collapsible:* C problem 8; Choset's remark that for
   Gaussian noise this is the exact posterior; the sister book's completing-the-square route
   (`…/ch06-kalman-filters`) and the proof the two coincide.
4. **The Kalman filter is a Luenberger observer with a scheduled gain** (App. J.4 + §8.2.5; the
   chapter's lens). *Statement:* substituting update into prediction,
   $\hat x(k{+}1 \mid k{+}1) = F\hat x(k \mid k) + Gu(k) + R(k{+}1)[\,y(k{+}1) - H(F\hat x(k \mid k) + Gu(k))\,]$
   — a Luenberger observer with gain $R(k{+}1)$ recomputed from $P$; with a fixed gain $K$,
   $e(k{+}1) = (I - KH)F\,e(k) + \text{noise}$, stable iff the eigenvalues of $(I - KH)F$ lie in the
   unit disc (Thm. J.5.1). *Sketch:* (i) write the observer; (ii) subtract the true dynamics;
   (iii) collect $e$; (iv) read off the eigenvalue condition; (v) the KF picks, among all gains,
   the one minimizing $\operatorname{tr} P^+$ each step. *Collapsible:* eigenvalue placement (Thm.
   J.3.2 transposed; Appendix E); the steady-state gain as the Riccati fixed point — the bridge
   from "knob" to "schedule" that w15.1 animates.
5. **Observability rank test** (Thm. 8.2.2; Thm. J.4.1; J.5.2). *Statement:* $(F, H)$ is
   observable iff $\operatorname{rank}[H; HF; \dots; HF^{n-1}] = n$; then the KF error is bounded
   for all time. *Sketch:* (i) with zero noise stack $y(0..n{-}1) = Q\,x(0) + $ known input terms;
   (ii) $x(0)$ is recoverable iff $Q$ is injective; (iii) higher powers add nothing by
   Cayley–Hamilton; (iv) $(F, H)$ observable iff $(F^\top, H^\top)$ controllable. *Collapsible:*
   the dead-reckoning case below — "the problem lies with the system, not the filter" (§8.2.7); for
   nonlinear systems the test is applied to the linearized pair and is only local.
6. **EKF for range-and-bearing localization** (§8.3, §8.3.1). *Statement:* for
   $x = [x_r, y_r, \theta_r]^\top$, $f(x,u) = [x_r + u_1\cos\theta_r,\ y_r + u_1\sin\theta_r,\
   \theta_r + u_2]^\top$ and $h_j(x) = [\rho_j,\ \operatorname{atan2}(y_r - y_{\ell j}, x_r - x_{\ell j})
   - \theta_r]^\top$, the EKF is the KF with $F = \partial f/\partial x|_{\hat x(k|k)}$,
   $H_j = \partial h_j/\partial x|_{\hat x(k+1|k)}$ (eqs. 8.37–8.45). *Sketch:* (i) first-order
   Taylor expansion about the estimate; (ii) $F = \begin{bmatrix}1&0&-u_1\sin\theta_r\\0&1&u_1\cos\theta_r\\0&0&1\end{bmatrix}$;
   (iii) $H_j$ rows: range $[\Delta x/\rho,\ \Delta y/\rho,\ 0]$, bearing
   $[-\Delta y/\rho^2,\ \Delta x/\rho^2,\ -1]$, $\Delta = \hat x - x_{\ell j}$ (Choset's
   unsimplified atan2 derivative, simplified; the test checks both forms agree); (iv) stack visible
   landmarks; (v) range-only = drop bearing rows (§8.3.3). *Collapsible:* wrapping the bearing
   innovation; sister `ch07-ekf-ukf-manifolds` (UKF, error-state EKF on $SE(2)$) and
   `ch11-localization-gaussian` (gating, MHT).

**Named algorithms** (Choset numbers none in Ch. 8; names follow his section titles):

| Algorithm | Signature | Complexity per step |
|---|---|---|
| `simple_observer` (§8.2.2) | `(x̂, u, y; F, G, H) -> x̂` | $O(n^2 + p^3)$ |
| `kalman_filter` (§8.2.5, eqs. 8.26–8.32) | `(x̂, P, u, y; F, G, H, V, W) -> (x̂, P)` | $O(n^3)$; $O(n^2 p + p^3)$ for $p \ll n$ |
| `observability_rank` (Thm. 8.2.2) | `(F, H) -> rank Q` | one SVD of an $np \times n$ matrix |
| `ekf` (§8.3, eqs. 8.37–8.45) | `(x̂, P, u, y; f, h, ∂f, ∂h, V, W) -> (x̂, P)` | as KF plus Jacobians |
| `associate_ml` (§8.3.2) | `(y_i, x̂⁻, P⁻, landmarks) -> a(i)` | $O(n_\ell\, p^3)$ per measurement |
| `kalman_slam` (§8.4.1; §8.4.2) | `(x̂, P, u, y) -> (x̂, P)`, $n = 3 + 2n_\ell$ | $O(n_\ell^2)$; $O(n_\ell^3)$ naive |

**Numeric micro-example** (the contract with the unit test in §5): Choset's dead-reckoning robot
(§8.2.6): $T = 0.5$, $m = 1$, $F = \begin{bmatrix}1&0.5\\0&1\end{bmatrix}$, $G = [0,\, 0.5]^\top$,
$u = 0$, $V = \begin{bmatrix}0.2&0.05\\0.05&0.1\end{bmatrix}$, $H = [0\ 1]$ (velocity sensor),
$W = 0.5$, $\hat x(k \mid k) = [2, 4]^\top$, $P(k \mid k) = \operatorname{diag}(1, 2)$. Choset
reports no measurement value; the chapter fixes $y(k{+}1) = 2.7$, $y(k{+}2) = 2.0$.
- *Step 1 predict:* $\hat x^- = [4, 4]^\top$, $P^- = \begin{bmatrix}1.7&1.05\\1.05&2.1\end{bmatrix}$.
  *Update:* $\nu = -1.3$, $S = 2.6$, $R = [0.4038, 0.8077]^\top$, so $\nu/S = -0.5$ exactly and
  $\hat x^+ = [3.475, 2.95]^\top$; $P^+ = P^- - R\,[1.05\ \ 2.1] = \begin{bmatrix}1.2760&0.2019\\0.2019&0.4038\end{bmatrix}$.
- *Step 2 predict:* $\hat x^- = [4.95, 2.95]^\top$, $P^- = \begin{bmatrix}1.7788&0.4538\\0.4538&0.5038\end{bmatrix}$.
  *Update:* $\nu = -0.95$, $S = 1.0038$, $R = [0.4521, 0.5019]^\top$, $\hat x^+ = [4.520, 2.473]^\top$,
  $P^+ = \begin{bmatrix}1.5737&0.2261\\0.2261&0.2510\end{bmatrix}$.
- *Reading:* velocity variance $2 \to 0.404 \to 0.251$ (the squish of Fig. 8.7); position variance
  $1 \to 1.276 \to 1.574$, growing through two optimal updates. *Rank test (2×2):*
  $Q = [H; HF] = \begin{bmatrix}0&1\\0&1\end{bmatrix}$, rank 1: unobservable. With a position sensor
  $H = [1\ 0]$: $Q = \begin{bmatrix}1&0\\1&0.5\end{bmatrix}$, rank 2. The test asserts every entry to
  $10^{-3}$ and both ranks.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, one line: **"a copy of the robot, tugged toward what the sensor saw — and the
filter decides how hard to tug."**

- **Widget w15.1: Observer Bench** — type: wasm-sim. Rusty dead-reckons along a 1D rail in the
  Apartment hallway (the §8.2.6 model; a second mode adds a door-corner range sensor).
  **Manipulates:** headline — the observer gain $K$ (one knob; eigenvalues of $(I - KH)F$ on a
  unit-disc inset); a "Kalman" toggle hands the knob to the filter and animates $R(k)$ converging;
  behind a disclosure — $V$, $W$, sensor choice (velocity / position), seed re-roll (`Pcg64`, seed
  visible). **Observes:** estimate (purple) vs. truth (gray dashed); the 95% ellipse in
  $(x_r, v_r)$ growing orange on predict, collapsing purple on update — Figs. 8.5–8.7 live; the
  green innovation strip; a trace-$P$ sparkline. **Misconception killed:** *"the Kalman gain is a
  tuning constant"* — it is a trajectory, and the knob the reader was turning is what the filter
  computes.
- **Widget w15.2: Observability Probe** — type: wasm-sim. Rusty drives a square in the Apartment
  with the §8.3.1 EKF against known landmarks. **Manipulates:** headline — visible landmarks
  (one / two / all) with a range-only switch; behind a disclosure — landmark drag handles,
  measurement noise, seed re-roll. **Observes:** with one range-only landmark the pose ellipse
  stretches along the constant-range circle and never shrinks there; with two landmarks everything
  collapses; a panel prints $\operatorname{rank}[H; HF; HF^2]$ of the *linearized* pair, colored by
  whether it is 3. **Misconception killed:** *"more iterations always mean more certainty"* — along
  an unobservable direction the optimal filter is optimally uncertain, forever.
- **Figure f15.3: Hyperplane and Ellipse** — type: static-svg (hover). Figs. 8.2–8.4 redrawn:
  prediction ellipse (orange), $\Omega$ (green), Euclidean vs. Mahalanobis foot, then $\Omega^*$.
  **Misconception killed:** *"the update moves perpendicular to the measurement"* — perpendicular
  in the covariance metric, which is why uncertain directions move more.

Roles: estimation colors (`--pr-*`) plus `--rm-robot` for the executed trajectory and
`--rm-obstacle` for walls; landmarks use the chrome accent. Dashboard layout: w15.1 full-width atop
the C section with the Choset/Thrun symbol legend docked under it; f15.3 half-width beside the
§8.2.3–8.2.4 derivations, sticky while they scroll; w15.2 full-width closing §8.3 with the rank
readout under the ellipse. Every widget autoplays a seeded default and degrades to a static SVG.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`SMatrix`/`SVector`; `DMatrix` only for the observability matrix,
whose row count $np$ is not a const expression), `rand` 0.9 (`Pcg64`), the Ch. 2 ports `sim`,
`geom`, `prob`, and the sister book's filter crates vendored unchanged as the `filters` port (this
chapter extends Ch. 2's port list): `Kf<N, U, M>` (its Ch. 6), `Ekf` + `MotionModel` /
`MeasurementModel` (its Ch. 7), `EkfLocalizer`/`ml_associate` (its Ch. 11), `EkfSlam` (its Ch. 14).
No filter is re-implemented; `adskalman` appears only inside the port's own cross-validation tests.

**Module plan** — `crates/estimate/` (thin: every file is an adapter or a piece Choset has and the
sister book does not): `src/lti.rs` (`Lti<N, U, M>`, `discretize` per J.5, `observability_matrix`,
`is_observable`); `src/observer.rs` (fixed-gain Luenberger `Observer`, `error_eigs`); `src/kf.rs`
(Choset-notation `kalman_step` over `filters::Kf`; `dead_reckoning()` constants);
`src/landmarks.rs` (`RangeBearing`, `RangeOnly` as sister `MeasurementModel`s; `associate_ml` with
the §8.4.2 new-landmark threshold); `src/slam.rs` (`simple_slam_2()` as `Kf<6, 2, 4>`; `KalmanSlam`
newtype over the sister's `EkfSlam`); `examples/dead_reckoning.rs`, `examples/one_landmark.rs`;
`demos/ch15-widgets/`.

**Key types & signatures**

```rust
// crates/estimate/src/lti.rs
use nalgebra::{SMatrix, SVector, DMatrix};

/// Choset's letters on purpose; each field names the Thrun symbol so readers of both books are
/// never one rename away from a bug (Choset's R is the *gain*).
pub struct Lti<const N: usize, const U: usize, const M: usize> {
    pub f: SMatrix<f64, N, N>,   // A_t
    pub g: SMatrix<f64, N, U>,   // B_t
    pub h: SMatrix<f64, M, N>,   // C_t
    pub v: SMatrix<f64, N, N>,   // R_t (process covariance)
    pub w: SMatrix<f64, M, M>,   // Q_t (measurement covariance)
}
impl<const N: usize, const U: usize, const M: usize> Lti<N, U, M> {
    /// Thm. 8.2.2: stack H, HF, …, HF^{N-1}. Dynamic because N*M is not a const expression.
    pub fn observability_matrix(&self) -> DMatrix<f64>;
    /// rank(Q) == N under a relative SVD tolerance — a parameter, not a magic number.
    pub fn is_observable(&self, tol: f64) -> bool;
}

// crates/estimate/src/observer.rs — fixed-gain Luenberger observer (J.4, discretized by J.5):
// the knob the Kalman filter turns. x̂⁺ = F x̂ + G u + K (y − H(F x̂ + G u)).
pub struct Observer<const N: usize, const U: usize, const M: usize> {
    pub sys: Lti<N, U, M>, pub k: SMatrix<f64, N, M>, pub xhat: SVector<f64, N>,
}
impl<const N: usize, const U: usize, const M: usize> Observer<N, U, M> {
    pub fn step(&mut self, u: &SVector<f64, U>, y: &SVector<f64, M>);
    pub fn error_eigs(&self) -> Vec<nalgebra::Complex<f64>>;  // of (I − K H) F; unit disc ⇔ stable
}

// crates/estimate/src/kf.rs
pub use filters::kalman::{Gaussian, Kf};   // sister Ch. 6; not re-derived, not re-implemented
/// One Choset step (eqs. 8.26–8.32) through the sister's `Kf`; returns (ν, S) because data
/// association and the widgets need them.
pub fn kalman_step<const N: usize, const U: usize, const M: usize>(
    sys: &Lti<N, U, M>, bel: &mut Gaussian<N>, u: &SVector<f64, U>, y: &SVector<f64, M>,
) -> (SVector<f64, M>, SMatrix<f64, M, M>);
/// §8.2.6 constants: T = 0.5, m = 1, V, W = 0.5, x̂ = [2, 4], P = diag(1, 2).
pub fn dead_reckoning() -> (Lti<2, 1, 1>, Gaussian<2>);

// crates/estimate/src/landmarks.rs
/// h_j = [ρ, atan2(Δy, Δx) − θ]; Jacobian rows [Δx/ρ, Δy/ρ, 0], [−Δy/ρ², Δx/ρ², −1].
pub struct RangeBearing<'m> { pub map: &'m [Landmark] }
pub struct RangeOnly<'m>    { pub map: &'m [Landmark] }   // §8.3.3: bearing row removed
impl filters::nonlinear::MeasurementModel<geom::SE2, 3, 2> for RangeBearing<'_> { /* h, H */ }
/// §8.3.2: a(i) = argmin_j ν_ij^T S_ij^{-1} ν_ij; `None` above `chi2_new` ⇒ initialize a landmark.
pub fn associate_ml(y_i: &SVector<f64, 2>, bel: &Gaussian<3>, map: &[Landmark],
                    w: &SMatrix<f64, 2, 2>, chi2_new: f64) -> Option<usize>;

// crates/estimate/src/slam.rs
/// §8.4.1, two landmarks: state [x_r, y_r, x_ℓ1, y_ℓ1, x_ℓ2, y_ℓ2]; linear, so plain `Kf`.
pub fn simple_slam_2(v_r: SMatrix<f64, 2, 2>, w_i: SMatrix<f64, 2, 2>) -> Lti<6, 2, 4>;
/// §8.4.2 is the sister's EKF-SLAM; this newtype renames the API into Choset's words.
pub struct KalmanSlam(pub filters::slam::EkfSlam);
```

**Worked end-to-end example:** `cargo run --example dead_reckoning -p estimate` builds
`dead_reckoning()`, feeds $u = 0$ with $y = 2.7$ then $2.0$, prints both steps in the §3 layout,
then `rank Q = 1 (velocity sensor), 2 (position sensor)`.
`#[test] fn reproduces_choset_8_2_6_two_steps()` asserts $\hat x$, $P$ to $10^{-3}$ and both ranks;
`#[test] fn kf_is_observer_with_scheduled_gain()` checks that `Observer` with `k` set to the KF
gain each step reproduces the KF mean bit-for-bit; `#[test] fn matches_sister_kf()` runs the same
sequence through the vendored `filters::Kf` directly — the TOC's cross-validation.

**Runnable artifact:** `estimate` compiles to WASM as w15.1 and w15.2; the Bench's knob literally
sets `Observer::k`, its "Kalman" toggle swaps in `kalman_step`. The Integration lab drives Rusty
from the Ch. 2 `sim` with real encoder and range-bearing noise through `RangeBearing` and the
vendored `Ekf`, then forgets the landmarks and runs `KalmanSlam` on the same log; its belief panel
is the one Ch. 16 and the capstone import.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w15.1 | Observer Bench | wasm-sim | estimate + filters (port) + sim, seeded Pcg64 | gain knob, Kalman toggle, V/W, sensor choice, seed re-roll | KF = Luenberger observer with a scheduled gain; predict grows, update squishes |
| w15.2 | Observability Probe | wasm-sim | estimate + filters::Ekf + sim | landmark count, range-only, drag landmarks, seed re-roll | unobservable directions never shrink; rank test on the linearized pair |
| f15.3 | Hyperplane and Ellipse | static-svg (hover) | build-time SVG | hover Ω / Ω* / both feet | Euclidean vs. Mahalanobis projection; the most likely output |
| — | Dead-reckoning drift hook | animation (autoplay) | sim | replay only | why an observer is needed (Fig. 8.1 on Rusty) |

## 7. Exercises & Extensions

1. **(F)** Carry out C problem 7: from $P^+ = E[(x - \hat x^+)(x - \hat x^+)^\top]$ and
   $\hat x^+ = \hat x^- + R\nu$ derive $P^+ = P^- - RHP^-$, marking where §8.2.3's noiseless-output
   assumption is used.
2. **(F)** Prove Thm. 8.2.1 via C problem 8, then show the sister's gain
   $K = \bar\Sigma C^\top(C\bar\Sigma C^\top + Q)^{-1}$ and Choset's $R$ coincide under the §3 map.
3. **(C, predict-then-verify)** In w15.1 with default $V, W$, predict the steady-state gain for
   the velocity sensor and the eigenvalues of $(I - KH)F$ there; switch to the position sensor and
   predict which component of $P$ now shrinks. Verify; reconcile with the two rank computations.
4. **(C)** In w15.2 place two landmarks and the robot on one line with range-only sensing. Predict
   the ellipse shape, verify, and explain with the linearized rank test why collinearity is the trap.
5. **(P)** Implement `Lti::steady_state_gain` by iterating the Riccati recursion; test that
   `Observer` with that gain and the full KF agree to $10^{-6}$ after 50 steps on §8.2.6.
6. **(P, stretch)** Implement C problem 9 (unicycle, position-only sensor,
   $\hat x(1|1) = [1, 0.5, \pi/4]^\top$, $u = [3, \pi]$, $T = 0.25$, $P = I$, $y(2) = 1.7$) as an EKF
   unit test printing $\hat x(2|2)$, $P(2|2)$; run it through the sister's UKF and report the gap.

## 8. Modernization Notes

- **Kept, and why:** Choset's observer-first derivation (§8.2.2 → 8.2.3 → 8.2.4) in full — the one
  presentation that makes the gain *geometric* and ties estimation to the control theory a planner
  already uses; the §8.2.6 example; Thm. 8.2.2 with App. J.4–J.5 behind it; the range-bearing and
  range-only Jacobians; Mahalanobis association and the new-landmark threshold; Kalman SLAM in its
  2005 form, attributed to Smith, Self, and Cheeseman [C ref. 390].
- **Added beyond 2005:** (i) "KF = Luenberger observer with a gain schedule" as a theorem and a
  widget (Choset implies it across §8.2 and App. J); (ii) NEES/NIS instrumentation from the sister
  book's Ch. 6 in w15.2; (iii) the documented $(R, V, W) \leftrightarrow (K, R, Q)$ symbol hazard;
  (iv) EKF-SLAM inconsistency as an observability artifact — Huang, Mourikis & Roumeliotis (verify)
  — and Barfoot's observer–estimator bridge (verify); (v) depth linked, not re-derived: completing
  the square and information form (sister Ch. 6), UKF and on-manifold EKF (Ch. 7), gating and MHT
  (Ch. 11), EKF-SLAM cost and inconsistency (Ch. 14), factor graphs (Ch. 15).
- **Dropped or condensed, and why:** App. I — the sister's Ch. 2 is the primer; App. J.2–J.3 —
  Appendix E; Figs. 8.5–8.7's narrative — w15.1; the stacked-$H$ derivation — `Kf<6, 2, 4>` + link.
- **Honesty items the prose must keep:** for non-Gaussian noise the KF is only the best *linear*
  estimator (Choset's caveat); the EKF is a linearization, not a posterior; the unobservable
  position variance in §8.2.6 grows under an *optimal* filter; the rank test for nonlinear systems
  is local; the worked example's measurement values are ours, not Choset's; Kalman SLAM as written
  is $O(n_\ell^2)$ per step and inconsistent on long loops — why Ch. 16 and the sister book move on.
