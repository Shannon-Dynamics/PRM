# Chapter 22 — Learning and Planning

> Part VI — Frontiers and Integration · Estimated length: 9 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Every planner in Parts II–V was designed by hand: uniform samples, Euclidean heuristics, Gaussian
and bridge biases a human reasoned out from the geometry of narrow passages. This chapter asks the
2026 question without hype: where does machine learning belong in a motion planner? The answer
that organizes everything: **learning replaces the *proposal*, never the *verifier*.** A learned
sampler changes *where* a PRM looks, not *whether* an edge is collision checked; a learned
heuristic changes the *order* A\* expands nodes, not the test that declares a path valid; a neural
planner that emits waypoints directly is only as complete as the classical repair behind it.
Choset's own completeness proof holds the door open — it ends by remarking that "the sampling
distribution is not necessarily uniform" (C §7.4.3) — and this chapter walks through it:
completeness and optimality are *inherited from the verifier*, speed is *bought from the learner*,
and the exchange rate is measurable. Two "aha"s: (1) a sampler mixed with a sliver of uniform mass
keeps probabilistic completeness for free, so a learned sampler can only lose speed — and KL
divergence from the path distribution measures how much it wins; (2) a learned heuristic can be
*more informed* and *worse* at once — the $\epsilon$ by which it overestimates is the exact bound
on the path cost you give up.

Story line:

1. **Hook:** Ch. 11's Workbench narrow passage — uniform PRM needs 3 000 samples; a sampler trained
   on 2 000 *other* Workbenches connects in 300 (autoplay). Then a Workbench from a different
   generator appears and the learned sampler wastes every sample on a corridor that is not there.
2. **Play (C):** w22.1 — same budget, where samples fall, when the query connects; a $\lambda$ disclosure.
3. **Formalize (F):** proposals; the bounded-below mixture and why Choset's $\ell e^{-pn}$ bound
   survives; maximum likelihood as KL minimization; $\epsilon$-admissibility and $(1+\epsilon)$-optimality.
4. **Algorithm:** conditional sampler (CVAE, after Ichter, Harrison & Pavone "(verify)"); cost-to-go
   regression labeled by Ch. 6 backward Dijkstra; MPNet-style planner with lazy repair;
   Diffuser-style denoising with cost guidance (statement-level).
5. **Implementation (P):** `learn` crate on `candle` — training native, inference a dependency-free
   forward pass; `LearnedSampler` implements Ch. 11's `Sampler`, `HeuristicNet` is the heuristic closure of
   Ch. 6's A\*; the same PRM and A\* code runs with either.
6. **Integration lab and close:** the Honest Benchmark Board — in-distribution and held-out
   Workbenches, baselines tuned with equal effort, wall-clock *including* inference — then the
   principled/alchemy line, with the circularity of training on the planner you benchmark against.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (Workbench generator, Reach's `Collision` impl — the data factory); Ch. 4
  (Reach's C-obstacles on $T^2$ — what the learned density is a density *over*); Ch. 5 (`trait
  Manifold`; wrap-aware encodings on $T^2$); Ch. 6 (A\*, admissibility/consistency proofs; backward
  Dijkstra = cost-to-go labels); Ch. 11 (`Sampler`, PRM and its completeness proof, kd-tree,
  `Steer`); Ch. 12 (RRT-Connect — the repair planner behind the neural planner); Ch. 13 (RRT\* —
  the optimality baseline a proposal must not pretend to beat); Ch. 19 (CHOMP's functional — the
  diffusion guidance term); Ch. 21 (hybrid A\* — host of the stretch exercise).
- **Feeds into:** Ch. 23 (a "learned sampler" toggle in Reach's panel; this chapter's benchmark
  cited in the retrospective).
- **Baseline sources:** NEW (TOC §3); Choset read for context, owned elsewhere — C §7.1.3 (Ch. 11;
  b. 216–217): OBPRM [C ref. 18], the Gaussian sampler [C ref. 59],
  the bridge test [C ref. 193] — hand-designed biased proposals, the direct ancestors of learned
  ones. C §7.4.3 (Ch. 11; b. 252–253): the failure bound $\ell(1-p)^n \le \ell e^{-pn}$ and the remark that
  the proof never used uniformity. C §7.5.6 (Ch. 14 pointer box; b. 262–266): energy-weighted acceptance of molecular
  configurations — a *designed* non-uniform sampler. C App. H.2 (Ch. 6; b. 528–532): optimistic
  (admissible) heuristics, Alg. 24 A\*, the informal optimality discussion F3 sharpens. Sister
  book Ch. 25 (Learning in the Loop — the same principled/alchemy discipline for estimation) and
  Ch. 20 (planning preview). Modernization set, every item **(verify)**: Ichter, Harrison & Pavone
  (ICRA 2018); Qureshi et al., MPNet (ICRA 2019; T-RO 2021); Bhardwaj, Choudhury & Scherer (CoRL
  2017) and Choudhury et al. (IJRR 2018); Janner et al., Diffuser (ICML 2022); Carvalho et al.,
  Motion Planning Diffusion (venue to verify).

## 3. Foundation (F) — Mathematical Core

**Notation introduced this chapter** (all other symbols per TOC §2):

| Symbol | Meaning |
|---|---|
| $\mathcal{E}$ | environment encoding (occupancy raster of the Workbench) fed to a net |
| $u(q) = 1/\mu(\mathcal{Q})$ | uniform density on $\mathcal{Q}$ (Ch. 11's default proposal) |
| $\hat p_\theta(q \mid \mathcal{E})$, $p^*(q \mid \mathcal{E})$ | learned proposal; the target — empirical distribution of nodes on Ch. 11 roadmap paths |
| $p_\lambda = (1-\lambda)\hat p_\theta + \lambda u$ | bounded-below mixture, $\lambda \in (0,1]$ |
| $D_{KL}(p \,\|\, q) = \sum_i p_i \ln(p_i/q_i)$ | KL divergence (nats; bits in parentheses) |
| $h^*(v)$, $\hat h_\theta(v)$, $\epsilon$ | true cost-to-go; learned heuristic; smallest $\epsilon$ with $\hat h_\theta \le (1+\epsilon)h^*$ |
| $\beta_k$, $\epsilon_\theta(\tau^k, k, \mathcal{E})$, $J(\tau)$ | diffusion schedule; noise predictor over a trajectory $\tau \in \mathcal{Q}^T$; guidance cost (Ch. 19) |

**Definitions** (each gets a display box):

- **D22.1 Sampling proposal.** Any density on $\mathcal{Q}$ a sampling-based planner draws
  candidates from; the *verifier* (node collision test, `Steer` on edges) is unchanged by it.
- **D22.2 Completeness-preserving proposal.** $p(q) \ge \lambda\, u(q)$ for all $q$ and a fixed
  $\lambda > 0$. Every $p_\lambda$ is one; a raw $\hat p_\theta$ generally is not (nets emit zeros).
- **D22.3 $\epsilon$-admissible heuristic.** $\hat h(v) \le (1+\epsilon)\, h^*(v)$ for every node;
  $\epsilon = 0$ is Choset's optimistic heuristic (App. H.2).
- **D22.4 Neural motion planner.** A waypoint policy $\pi_\theta$ iterated from $q_{start}$ toward
  $q_{goal}$ whose output passes through a classical *repair* (Ch. 11 `Steer` per hop; Ch. 12
  RRT-Connect between the endpoints of a failed hop).
- **D22.5 Diffusion planner.** A trajectory sampler realized by iterated denoising from Gaussian
  noise, optionally tilted by $\exp(-J(\tau))$; a *proposal* in the sense of D22.1, verified after.
- **D22.6 Honest benchmark.** Held-out worlds from a generator *different* from the training one;
  metrics: success at fixed budget, cost ratio to RRT\*'s 10 s incumbent, wall-clock including
  inference and amortized training; seeded classical baselines tuned with equal effort.

**Key derivations** (name · statement · sketch · collapsible):

1. **Probabilistic completeness survives any bounded-below proposal.** *Statement:* under
   $p_\lambda$, PRM's failure probability on an $(\epsilon,\alpha,\beta)$-expansive query obeys
   $\Pr[\text{fail}] \le \ell(1-\lambda p)^n \le \ell e^{-\lambda p n}$ with Choset's uniform
   constants. *Sketch (5 steps):* C §7.4.3 tiles a path by $\ell$ sets of uniform measure $\ge p$;
   failure needs an empty tile; under $p_\lambda$ each draw lands in a tile w.p. $\ge \lambda p$;
   union bound; only a *lower* bound on tile mass was used, as Choset remarks. *Collapsible:* Ch. 11's
   full proof with $\lambda$ carried through; why $\lambda = 0$ voids it.
2. **Hitting time and KL as the price of concentration.** *Statement:* expected draws to land in
   region $A$ is $1/p(A)$, so $\hat p(A) = \alpha\,u(A)$ hits it $\alpha\times$ sooner, capped at
   $1/(\lambda u(A))$ under the mixture. On a cell partition $D_{KL}(\hat p\|u) = \ln N_{cells} -
   H(\hat p)$ measures concentration; $D_{KL}(p^*\|\hat p_\theta)$ measures aim, and minimizing it
   over $\theta$ *is* maximum likelihood on path-node samples. *Sketch (5 steps):* geometric waiting
   time; mixture cap; expand with $u_i = 1/N$; write $\max_\theta \sum_j \ln \hat p_\theta(q_j)$ and
   subtract the constant $H(p^*)$; conclude. *Collapsible:* the CVAE evidence lower bound on $T^2$
   (Ichter et al. "(verify)"); why the latent $z$ lets one net propose *both* corridors of a world.
3. **$\epsilon$-admissible A\* is $(1+\epsilon)$-optimal.** *Statement:* with $\hat h \le
   (1+\epsilon)h^*$, Ch. 6's A\* with reopening returns cost $\le (1+\epsilon)C^*$. *Sketch (5
   steps):* suppose the goal pops at $C > (1+\epsilon)C^*$; an open node $v$ on an optimal path has
   $g(v) = g^*(v)$; $f(v) \le g^*(v) + (1+\epsilon)h^*(v) \le (1+\epsilon)C^* < C$; $v$ would pop
   first; contradiction. *Collapsible:* additive variant ($\hat h \le h^* + \delta \Rightarrow C \le
   C^* + \delta$), provenance (Pearl, *Heuristics* "(verify)"), the reopening caveat.
4. **Proposal + verifier inherits the verifier's guarantee.** *Statement:* if a learned stage emits
   a candidate and a classical stage certifies it or falls back to a planner with guarantee
   $\mathcal{G}$, the composition has guarantee $\mathcal{G}$; without the fallback its only property
   is an empirical success rate. *Sketch (4 steps):* case split; certified ⇒ valid; otherwise the
   fallback runs on the *original* query; $\mathcal{G}$ is a statement about that query; time bounds
   do not transfer. *Collapsible:* MPNet's lazy-repair schedule as the instance.
5. **Diffusion objective and guidance (statement-level).** *Statement:* with noising
   $q(\tau^k \mid \tau^{k-1}) = \mathcal{N}(\sqrt{1-\beta_k}\,\tau^{k-1}, \beta_k I)$, training
   minimizes $\mathbb{E}\|\epsilon - \epsilon_\theta(\tau^k, k, \mathcal{E})\|^2$; guided sampling adds
   $-\alpha_k \nabla_\tau J(\tau^k)$ per step, approximately sampling $p_\theta(\tau)\exp(-J(\tau))$.
   *Sketch (4 steps):* noising destroys structure; the net reverses one step; chaining samples the
   demonstrations (Ch. 13 RRT\* paths); guidance tilts toward low $J$ — but collision is a hard
   constraint, so Ch. 11's checker runs afterward. *Collapsible:* pointer-level ELBO only.

**Named algorithms** (signature · complexity):

| Algorithm | Signature | Complexity |
|---|---|---|
| `generate_dataset(gen, n_worlds, seed)` | Workbenches → C Alg. 6 BASIC PRM paths → $(\mathcal{E}, q)$; backward Dijkstra → $(\mathcal{E}, v, h^*)$ | $O(n_{worlds}(n\log n + \|V\|\log\|V\|))$ |
| `train_sampler(data, cfg)` | CVAE (2×64 encoder/decoder, latent 4), ELBO descent, native candle | $O(N_{pairs})$ per epoch |
| `LearnedSampler::sample(rng)` | Ch. 11 `Sampler`: w.p. $\lambda$ uniform, else decode $z \sim \mathcal{N}(0, I)$ | $O(c_{net})$ |
| `train_heuristic(data, cfg)` / `measure_epsilon(net, worlds)` | MLP $(\mathcal{E}, v) \mapsto \hat h$; $\max_v \hat h/h^* - 1$ on validation graphs | $O(N)$ per epoch; $O(\sum\|V\|)$ |
| `a_star_eps(G, s, g, ĥ)` | Ch. 6 A\* with reopening; returns path, expansions, $f$-trace | worst case Dijkstra |
| `mpnet_plan(env, q_s, q_g)` | ≤ $K$ hops of $\pi_\theta$; `lazy_repair` via `Steer` + RRT-Connect | $O(Kc_{net})$ + repair |
| `diffuser_sample(env, q_s, q_g, K)` | $K$ guided denoising steps; verify (native; replayed in w22.4) | $O(KTc_{net})$ |
| `benchmark(suite, planners, seed)` | the D22.6 table | $O(\text{suite} \times \text{budget})$ |

**Numeric micro-example** (the contract with the §5 tests; two parts):

*(a) KL between a learned and a uniform sampler.* Coarsen Reach's $T^2$ on one Workbench into 4
cells. Of $N = 200$ draws from `LearnedSampler` at $\lambda = 0$ the cells receive counts
$(100, 60, 30, 10)$, so $\hat p = (0.50, 0.30, 0.15, 0.05)$ against $u = (0.25, 0.25, 0.25, 0.25)$:
$$D_{KL}(\hat p\|u) = 0.5\ln 2 + 0.3\ln 1.2 + 0.15\ln 0.6 + 0.05\ln 0.2
= 0.34657 + 0.05470 - 0.07662 - 0.08047 = \mathbf{0.2442\ \text{nats}}\ (0.352\ \text{bits}).$$
The target from the same world's roadmap is $p^* = (0.55, 0.30, 0.10, 0.05)$: $D_{KL}(p^*\|\hat p) = 0.55\ln 1.1 +
0.10\ln(2/3) = 0.0119$ nats versus $D_{KL}(p^*\|u) = \ln 4 - H(p^*) = 1.38629 - 1.07005 = 0.3162$ —
about $27\times$ better aimed. At $\lambda = 0.1$ the mixture $p_\lambda = (0.475, 0.295, 0.160,
0.070)$ has minimum mass $0.070 \ge \lambda/4 = 0.025$, so D22.2 holds.

*(b) Admissibility of a learned heuristic on 3 nodes.* Chain $a \xrightarrow{2} b \xrightarrow{2} c
\xrightarrow{1} goal$, $h^* = (5.0, 3.0, 1.0)$, net output $\hat h = (4.6, 3.4, 0.8)$. Admissible at
$a$ and $c$; **violated at $b$** ($3.4 > 3.0$); consistency $\hat h(b) \le 2 + \hat h(c) = 2.8$ also
fails at $b$. $\epsilon = \max(0.92, 1.1333, 0.80) - 1 = \mathbf{0.1333}$; by F3 any path from $a$
costs at most $1.1333 \times 5.0 = 5.667$. w22.2 draws $b$ in red and prints exactly these numbers.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor in one line: *the learner is a scout who says "look here"; the planner is the
surveyor who still measures every edge.* Colors per TOC §2 — start **green**, goal **red**,
obstacle **slate**, C-obstacle **amber hatched**, samples/graph **blue**, path **purple**, robot
**orange**; learned densities are a monochrome heat layer under the blue samples so no data hue is
overloaded. Every stochastic widget shows its seed with a re-roll.

- **Widget w22.1: Learned Sampler vs Uniform** — wasm-sim (flagship). Two Ch. 11 PRMs grow side by
  side on the *same* Workbench and seed: left uniform, right `LearnedSampler`, its $\hat p_\theta$ a
  heat layer on Reach's unwrapped torus under amber C-obstacles. **Manipulates:** headline — sample
  budget $n$ (50–3 000); disclosure — $\lambda$, a "held-out generator" toggle, seed re-roll.
  **Observes:** samples falling, components merging, the frame at which the query connects (purple),
  live $D_{KL}(\hat p\|u)$ with the §3(a) histogram, success-vs-$n$ curves over repeated seeds.
  **Misconception killed:** "a learned sampler changes what the planner *can* find" — with
  $\lambda > 0$ both panes connect every solvable query; the learner moves the knee of the curve,
  and on the held-out world moves it the wrong way.
- **Widget w22.2: Heuristic Calibration** — wasm-sim (flagship). Left: a Workbench grid with A\*'s
  explored set (blue) and path (purple). Right: every node at $(h^*(v), \hat h_\theta(v))$ against
  the gray-dashed diagonal; nodes above it are red and lit on the map. **Manipulates:** headline —
  a scale $s \in [0.5, 1.5]$ multiplying $\hat h$; disclosure — heuristic picker (Euclidean / learned
  / learned-clipped), seed. **Observes:** expansions, cost ratio to Dijkstra, measured $\epsilon$,
  the F3 line $(1+\epsilon)C^*$ the realized cost never crosses. **Misconception killed:** "a more
  informed heuristic is always better" — the inadmissible cloud expands fewer nodes *and* returns a
  longer path.
- **Widget w22.3: Neural Planner Autopsy** — interactive animation (weights trained offline,
  replayed). An MPNet-style $\pi_\theta$ hops Reach across a training-distribution Workbench; each
  hop gets a `Steer` tick or cross, failed hops trigger a visible RRT-Connect repair (blue mini-tree).
  Tab two: a held-out Workbench with a shape the generator never produced. **Manipulates:** headline
  — training vs held-out tab; disclosure — step scrub, "repair off" toggle, seed. **Observes:** the
  hop pass/fail strip; path length by net vs by repair. **Misconception killed:** "the net learned
  to plan" — it learned the training distribution; F4 makes the demo complete.
- **Widget w22.4: Denoising Trajectory** — animation (offline-trained Diffuser-style model on
  Reach's $T^2$, replayed). 32 noisy joint-space trajectories denoise over $K$ steps; at step $K$
  each runs through Ch. 11's checker and is drawn purple (valid) or red (colliding).
  **Manipulates:** headline — step scrub; disclosure — guidance toggle, seed. **Observes:** fraction
  valid (high, never 1.0); both ways around an obstacle survive. **Misconception killed:**
  "generative models produce collision-free paths" — they produce *plausible* ones.
- **Widget w22.5: Honest Benchmark Board** — dashboard. The D22.6 table from recorded runs: rows =
  uniform / Gaussian / bridge / OBPRM / learned ($\lambda = 0.1$) / RRT-Connect / MPNet + repair;
  columns = success at budget, cost ratio to RRT\*, wall-clock without and with inference, for both
  suites. **Manipulates:** headline — suite toggle; disclosure — budget, an "amortize training over
  $k$ queries" slider. **Observes:** learned rows winning on one side and losing on the other; the
  break-even $k$. **Misconception killed:** "benchmarks without inference time and distribution
  shift are benchmarks."

Dashboard layout sketch (w22.5): `[toggle row: suite · budget · amortization] / [table: 7 planners ×
8 metrics, learned rows tinted orange] / [right: success-vs-budget and cost-ratio-vs-budget plots]`.

## 5. Practical (P) — Rust Implementation

**Crates** (per TOC stack): `candle-core`/`candle-nn` (TOC lists `candle` or `burn` without a
version; pin the minor at adoption, record `burn` as alternate; **native-only**); `safetensors`
(weight blobs — one tiny new dependency, recorded); `nalgebra` 0.35 (configurations and the
dependency-free forward pass); `rand` 0.9 (`SmallRng` for data, shuffles, sampling); `petgraph`
0.8 / `pathfinding` 4.15 (graph plumbing; `pathfinding` cross-checks `a_star_eps` at $\epsilon = 0$).
Imported artifacts: `sim`/`robots`/`collide` (Ch. 2), `cspace` (Ch. 4), `Manifold` (Ch. 5),
`search` (Ch. 6), `sampling` (Chs. 11–13), `trajopt::ChompProblem` (Ch. 19). **Design decision
(recorded):** inference never touches candle — each net is exported to `safetensors` and evaluated
by `infer.rs` (~40 lines, nalgebra only), so the WASM build and the `web/lib/learn/` TypeScript port
run the identical forward pass; a test asserts candle and `infer.rs` agree to $10^{-6}$ on 1 000
inputs. The diffuser is native-only; w22.4 replays traces.

**Module plan:** `crates/learn/`

```
src/
  data.rs           // randomized Workbenches (Ch. 2) + PRM path nodes (Ch. 11) + h* labels (Ch. 6)
  encode.rs         // EnvEncoding: 32×32 occupancy raster; (cos, sin) wrap features on T^2
  sampler.rs        // CVAE training (candle) + LearnedSampler: impl sampling::Sampler<Torus>
  heuristic.rs      // HeuristicNet training + heuristic closure for search::a_star; measure_epsilon
  neural_planner.rs // MpnetLike: waypoint policy + lazy_repair over Steer + RrtConnect
  diffusion.rs      // toy Diffuser over Q^T with CHOMP guidance (feature = "diffusion")
  infer.rs          // dependency-free MLP forward pass (WASM-safe)
  bench.rs          // D22.6 harness; emits the Board's JSON + SVG
demos/ch22-sampler/  demos/ch22-heuristic/   // w22.1, w22.2 (WASM); examples/learned_sampler.rs
```

**Key types & signatures:**

```rust
use manifold::Torus;                       // Ch. 5 (Reach's T^2)
use sampling::{Sampler, Hybrid, Uniform};  // Ch. 11 (Hybrid<A, B> is its mixture sampler)
use search::GridNode;                      // Ch. 6

pub struct EnvEncoding(pub nalgebra::SVector<f32, { 32 * 32 }>);
pub struct MlpWeights { layers: Vec<(nalgebra::DMatrix<f32>, nalgebra::DVector<f32>)> }

/// The raw learned proposal (D22.1). Never shipped alone — see `SafeLearnedSampler`.
pub struct LearnedSampler { decoder: MlpWeights, env: EnvEncoding }
impl Sampler<Torus> for LearnedSampler {
    fn sample<R: rand::Rng>(&self, rng: &mut R) -> Torus { /* decode z ~ N(0, I) through the net */ }
}
/// D22.2 by construction: Ch. 11's own `Hybrid` mixture with `Uniform` as the floor at weight
/// lambda; F1's bound needs lambda > 0, and the constructor rejects lambda = 0.
pub type SafeLearnedSampler = Hybrid<LearnedSampler, Uniform>;

/// Learned cost-to-go; `epsilon` is *measured* (measure_epsilon), never assumed.
pub struct HeuristicNet { net: MlpWeights, env: EnvEncoding, pub epsilon: Option<f64> }
impl HeuristicNet { pub fn h(&self, v: &GridNode, goal: &GridNode) -> f64; } // closure for Ch. 6 a_star

pub struct CoarseHistogram<const N: usize>(pub [u64; N]);
impl<const N: usize> CoarseHistogram<N> {
    pub fn kl_to_uniform(&self) -> f64;              // §3(a): D_KL(p_hat || u), nats
    pub fn kl_from(&self, target: &[f64; N]) -> f64; // D_KL(target || p_hat)
}

/// D22.4: the repair planner is a field, not an Option — F4 says the guarantee lives there.
pub struct MpnetLike<E: sampling::tree::Extend<Torus>> { policy: MlpWeights, repair: sampling::tree::RrtConnect<Torus, E>, max_hops: usize }
```

**Worked end-to-end example** (`cargo run --release --example learned_sampler -- --seed 7`):
generates 2 000 randomized Workbenches, builds a Ch. 11 PRM ($n = 400$, $k = 8$) on each, extracts
~60 000 path nodes and 2 000 backward-Dijkstra label grids; trains the sampler (~40 s native) and
heuristic (~25 s); prints (i) the §3(a) histogram on Workbench #0 — `counts [100, 60, 30, 10]
KL(p_hat||u) = 0.2442 nats  KL(p*||p_hat) = 0.0119  KL(p*||u) = 0.3162`; (ii) the §3(b) check —
`h* [5.0 3.0 1.0]  h_hat [4.6 3.4 0.8]  violated {b}  epsilon 0.1333  bound 5.667`; (iii) the Board
on 200 in-distribution and 200 held-out worlds. Pattern recorded for the suite: the learned sampler
wins success-at-budget in distribution, loses to the bridge sampler held-out, and the gap closes as
$\lambda \to 0.3$; the learned heuristic halves expansions at measured $\epsilon \approx 0.08$ and the
realized cost ratio never exceeds $1+\epsilon$ (a property test, not a hope). Tests:
`kl_four_cell_matches_hand_computation` (0.2442 ± 1e-4 plus the entropy identity),
`eps_admissibility_three_nodes` ($\epsilon = 0.1333$, violation set $\{b\}$),
`mixture_never_below_lambda_uniform`, `a_star_eps_cost_within_bound` (500 seeded grids),
`infer_matches_candle`.

**Runnable artifact:** `learn` compiles to WASM for w22.1 and w22.2 (forward passes only, weights
embedded); w22.3/w22.4 replay recorded traces; w22.5 renders recorded JSON. Integration lab: Reach
on the Workbench throughout; the stretch exercise brings Hitch and the Lot in. Both micro-example
numbers are invariants in `web/lib/__checks__.ts`.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w22.1 | Learned Sampler vs Uniform | wasm-sim | learn (infer), sampling (Ch. 11 PRM), cspace, sim, widget-kit | budget slider; $\lambda$, held-out toggle, seed | proposals buy speed, not completeness; shift reverses the gain |
| w22.2 | Heuristic Calibration | wasm-sim | learn (infer), search (Ch. 6 A\*), widget-kit | scale slider; heuristic picker, seed | informed ≠ admissible; $\epsilon$ bounds the loss exactly |
| w22.3 | Neural Planner Autopsy | interactive animation | recorded traces; sampling Steer/RrtConnect replay | train/held-out tab; scrub, repair toggle | the repair planner is the guarantee; the net is the prior |
| w22.4 | Denoising Trajectory | animation | recorded denoising traces; collide (Ch. 2) | step scrub; guidance toggle, seed | generative = plausible, not verified |
| w22.5 | Honest Benchmark Board | dashboard | bench JSON, plotting kit | suite toggle; budget, amortization | inference time and held-out worlds are part of the benchmark |

## 7. Exercises & Extensions

1. **(F)** Carry $\lambda$ through Ch. 11's full $(\epsilon,\alpha,\beta)$-expansiveness proof (not
   only F1's tiling bound) and name the one constant that changes. Then show that a proposal with
   $\hat p(A) = 0$ on a tile $A$ makes the failure probability constant in $n$.
2. **(F)** Prove the additive form of F3 and explain why neither form survives A\* *without*
   reopening when $\hat h$ is inconsistent; build a 4-node counterexample.
3. **(C, predict-then-verify)** In w22.1 set $\lambda = 0$ and switch to the held-out generator.
   Before pressing play, predict the success rate at $n = 1\,000$ and whether it ever reaches
   uniform's. Verify over 20 seeds.
4. **(P)** Implement `Sampler` on Rusty's $\mathbb{R}^2$ configurations for a `LearnedSampler` on the Apartment ($64 \times 64$
   raster, 500 randomized floorplans) and add the row to the Board. Does the held-out gap shrink or
   grow with the larger raster?
5. **(P, stretch)** Train a heuristic for Ch. 21's hybrid A\* on the Lot: labels from backward
   Dijkstra on the $(x, y, \theta)$ lattice. Measure $\epsilon$ on held-out Lots; compare realized
   cost ratio to the F3 bound and to the classical $\max(h_{RS}, h_{holonomic})$.

## 8. Modernization Notes

- **Kept from Choset, and why:** biased proposals are not a 2018 invention. §7.1.3 (b. 216–217)
  designs three — OBPRM [C ref. 18], the Gaussian sampler [C ref. 59], the bridge test [C ref. 193];
  §7.5.6 (b. 266) biases molecular roadmaps by an explicit energy acceptance probability; §7.4.3
  (b. 253) remarks that uniformity was never used. Learned sampling is framed as *automating the
  design of §7.1.3's proposals*, and every one of them stays as a baseline. App. H.2's optimistic
  heuristic (b. 528) is kept verbatim as D22.3 at $\epsilon = 0$.
- **Added beyond 2005 (each "(verify)"):** CVAE sampling distributions (Ichter, Harrison & Pavone,
  ICRA 2018); learned heuristics / search as sequential decision making (Bhardwaj, Choudhury &
  Scherer, CoRL 2017; Choudhury et al., IJRR 2018); neural planners with classical repair (Qureshi
  et al., MPNet, ICRA 2019 / T-RO 2021); trajectory diffusion with guidance (Janner et al.,
  Diffuser, ICML 2022; Carvalho et al., Motion Planning Diffusion — venue to verify); the
  $(1+\epsilon)$ bound's provenance (Pearl, *Heuristics*, 1984 — verify edition). The
  honest-benchmark discipline (D22.6) is the book's policy, not a citation.
- **Dropped / condensed:** reinforcement-learned policies (the sister book's Chs. 21–22 own decision
  making; a learned *policy* is not a planner proposal); large pretrained and language-conditioned
  planners (no small-net, in-browser story); sim-to-real (no real robot, said plainly).
- **Honesty ledger (the closing section the prose must keep):** *principled* — any
  completeness-preserving proposal (F1); ML as KL minimization (F2); $\epsilon$-admissibility with
  $\epsilon$ measured on validation data and its in-distribution caveat said aloud (F3); proposal +
  verifier inheritance (F4); benchmarks that include inference time and held-out worlds (D22.6).
  *Still alchemy in 2026* — generalization across world generators (our held-out gap is the
  evidence); validation $\epsilon$ is not a deployment guarantee; neural planners' success rates
  without repair; diffusion guidance through a non-differentiable collision constraint; the
  circularity that our sampler is trained on paths from the very PRM it is benchmarked against, so
  it can at best reproduce PRM-with-infinite-budget faster; and amortization — a 40-second training
  run is paid for only by enough queries on *similar* worlds, and the Board's slider shows the
  break-even.
