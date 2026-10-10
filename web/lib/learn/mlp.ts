/**
 * A dense multilayer perceptron with hand-written backpropagation.
 *
 * The Rust crate trains on `candle` and ships weights as `safetensors`; the
 * browser needs only the forward pass, and this port needs the backward pass
 * too so the chapter's checks can *train* (XOR, a 1-D function, the toy
 * denoiser) and so the whole learning story runs with no dependency. The net is
 * deliberately tiny: every chapter claim is about *where learning goes in a
 * planner*, not about network architecture, and a two-layer tanh net is enough
 * to make a sampler aim, a heuristic over-estimate, and a policy hop.
 *
 * Conventions: layer l maps nIn → nOut with weights stored row-major
 * (out × in); activations are applied after every layer except the last, which
 * gets `output`. Gradients accumulate into `gw`/`gb` until an optimizer step.
 * Everything is seeded through Chapter 11's `Rng` — never `Math.random`.
 */

import type { Rng } from '../prob/rng';

export type Activation = 'tanh' | 'relu' | 'linear' | 'sigmoid' | 'softplus';

export interface LayerJson {
  w: number[];
  b: number[];
}

export interface MlpJson {
  sizes: number[];
  hidden: Activation;
  output: Activation;
  layers: LayerJson[];
}

interface Layer {
  nIn: number;
  nOut: number;
  w: Float64Array;
  b: Float64Array;
  gw: Float64Array;
  gb: Float64Array;
}

function act(kind: Activation, z: number): number {
  switch (kind) {
    case 'tanh':
      return Math.tanh(z);
    case 'relu':
      return z > 0 ? z : 0;
    case 'sigmoid':
      return 1 / (1 + Math.exp(-z));
    case 'softplus':
      return z > 30 ? z : Math.log1p(Math.exp(z));
    case 'linear':
      return z;
  }
}

/** Derivative expressed through the activation value a (and z where needed). */
function actPrime(kind: Activation, z: number, a: number): number {
  switch (kind) {
    case 'tanh':
      return 1 - a * a;
    case 'relu':
      return z > 0 ? 1 : 0;
    case 'sigmoid':
      return a * (1 - a);
    case 'softplus':
      return 1 / (1 + Math.exp(-z));
    case 'linear':
      return 1;
  }
}

export class Mlp {
  readonly layers: Layer[] = [];
  /** Pre-activations and activations of the last `forward(x, true)` call, per layer. */
  private zs: Float64Array[] = [];
  private as: Float64Array[] = [];
  private lastX: Float64Array | null = null;

  constructor(
    readonly sizes: readonly number[],
    readonly hidden: Activation = 'tanh',
    readonly output: Activation = 'linear',
    rng?: Rng,
  ) {
    if (sizes.length < 2) throw new Error('Mlp: need at least an input and an output size');
    for (let l = 0; l + 1 < sizes.length; l++) {
      const nIn = sizes[l];
      const nOut = sizes[l + 1];
      const w = new Float64Array(nIn * nOut);
      const b = new Float64Array(nOut);
      if (rng) {
        // Glorot-uniform: keeps tanh pre-activations O(1) at init regardless of width.
        const limit = Math.sqrt(6 / (nIn + nOut));
        for (let i = 0; i < w.length; i++) w[i] = rng.uniform(-limit, limit);
      }
      this.layers.push({ nIn, nOut, w, b, gw: new Float64Array(nIn * nOut), gb: new Float64Array(nOut) });
    }
  }

  get nParams(): number {
    return this.layers.reduce((s, l) => s + l.w.length + l.b.length, 0);
  }

  get nIn(): number {
    return this.sizes[0];
  }

  get nOut(): number {
    return this.sizes[this.sizes.length - 1];
  }

  /** y = f(x). With `cache`, keep the per-layer values `backward` needs. */
  forward(x: ArrayLike<number>, cache = false): Float64Array {
    if (x.length !== this.nIn) throw new Error(`Mlp.forward: expected ${this.nIn} inputs, got ${x.length}`);
    let a = Float64Array.from(x as ArrayLike<number>);
    if (cache) {
      this.lastX = a;
      this.zs = [];
      this.as = [];
    }
    const L = this.layers.length;
    for (let l = 0; l < L; l++) {
      const { nIn, nOut, w, b } = this.layers[l];
      const kind = l === L - 1 ? this.output : this.hidden;
      const z = new Float64Array(nOut);
      const out = new Float64Array(nOut);
      for (let o = 0; o < nOut; o++) {
        let s = b[o];
        const row = o * nIn;
        for (let i = 0; i < nIn; i++) s += w[row + i] * a[i];
        z[o] = s;
        out[o] = act(kind, s);
      }
      if (cache) {
        this.zs.push(z);
        this.as.push(out);
      }
      a = out;
    }
    return a;
  }

  /**
   * Backpropagate dL/dy from the last cached forward pass; accumulate dL/dW,
   * dL/db into the layers' gradient buffers and return dL/dx (the chapter's
   * diffusion guidance needs a gradient with respect to the *input*).
   */
  backward(dy: ArrayLike<number>): Float64Array {
    if (!this.lastX) throw new Error('Mlp.backward: call forward(x, true) first');
    const L = this.layers.length;
    let delta = Float64Array.from(dy as ArrayLike<number>);
    for (let l = L - 1; l >= 0; l--) {
      const { nIn, nOut, w, gw, gb } = this.layers[l];
      const kind = l === L - 1 ? this.output : this.hidden;
      const z = this.zs[l];
      const a = this.as[l];
      const prev = l === 0 ? this.lastX : this.as[l - 1];
      // dL/dz = dL/da · a'(z)
      for (let o = 0; o < nOut; o++) delta[o] *= actPrime(kind, z[o], a[o]);
      const dx = new Float64Array(nIn);
      for (let o = 0; o < nOut; o++) {
        const d = delta[o];
        if (d === 0) continue;
        gb[o] += d;
        const row = o * nIn;
        for (let i = 0; i < nIn; i++) {
          gw[row + i] += d * prev[i];
          dx[i] += d * w[row + i];
        }
      }
      delta = dx;
    }
    return delta;
  }

  /** Gradient of a scalar function of the output with respect to the input only (weights untouched). */
  inputGradient(x: ArrayLike<number>, dy: ArrayLike<number>): Float64Array {
    this.forward(x, true);
    const L = this.layers.length;
    let delta = Float64Array.from(dy as ArrayLike<number>);
    for (let l = L - 1; l >= 0; l--) {
      const { nIn, nOut, w } = this.layers[l];
      const kind = l === L - 1 ? this.output : this.hidden;
      const z = this.zs[l];
      const a = this.as[l];
      const dx = new Float64Array(nIn);
      for (let o = 0; o < nOut; o++) {
        const d = delta[o] * actPrime(kind, z[o], a[o]);
        if (d === 0) continue;
        const row = o * nIn;
        for (let i = 0; i < nIn; i++) dx[i] += d * w[row + i];
      }
      delta = dx;
    }
    return delta;
  }

  zeroGrad(): void {
    for (const l of this.layers) {
      l.gw.fill(0);
      l.gb.fill(0);
    }
  }

  /** Plain SGD: θ ← θ − lr · g / n (n = samples accumulated). */
  sgdStep(lr: number, n = 1): void {
    const s = lr / n;
    for (const l of this.layers) {
      for (let i = 0; i < l.w.length; i++) l.w[i] -= s * l.gw[i];
      for (let i = 0; i < l.b.length; i++) l.b[i] -= s * l.gb[i];
    }
    this.zeroGrad();
  }

  /** Flat parameter vector (for checks and for the JSON export). */
  parameters(): Float64Array {
    const out = new Float64Array(this.nParams);
    let k = 0;
    for (const l of this.layers) {
      out.set(l.w, k);
      k += l.w.length;
      out.set(l.b, k);
      k += l.b.length;
    }
    return out;
  }

  gradients(): Float64Array {
    const out = new Float64Array(this.nParams);
    let k = 0;
    for (const l of this.layers) {
      out.set(l.gw, k);
      k += l.gw.length;
      out.set(l.gb, k);
      k += l.gb.length;
    }
    return out;
  }

  setParameters(theta: ArrayLike<number>): void {
    let k = 0;
    for (const l of this.layers) {
      for (let i = 0; i < l.w.length; i++) l.w[i] = theta[k++];
      for (let i = 0; i < l.b.length; i++) l.b[i] = theta[k++];
    }
  }

  clone(): Mlp {
    const m = new Mlp(this.sizes, this.hidden, this.output);
    m.setParameters(this.parameters());
    return m;
  }

  /** Weights rounded to `digits` significant figures — the browser's `safetensors`. */
  toJSON(digits = 5): MlpJson {
    const r = (x: number) => Number(x.toPrecision(digits));
    return {
      sizes: [...this.sizes],
      hidden: this.hidden,
      output: this.output,
      layers: this.layers.map((l) => ({ w: Array.from(l.w, r), b: Array.from(l.b, r) })),
    };
  }

  static fromJSON(j: MlpJson): Mlp {
    const m = new Mlp(j.sizes, j.hidden, j.output);
    j.layers.forEach((lj, l) => {
      m.layers[l].w.set(lj.w);
      m.layers[l].b.set(lj.b);
    });
    return m;
  }
}

/* -------------------------------------------------------------------------- */
/* Optimizers                                                                 */
/* -------------------------------------------------------------------------- */

/** Adam (Kingma & Ba): per-parameter step sizes, the default for every net in this chapter. */
export class Adam {
  private m: Float64Array;
  private v: Float64Array;
  private t = 0;

  constructor(
    readonly net: Mlp,
    /** Mutable so a schedule can decay it between epochs. */
    public lr = 1e-3,
    readonly beta1 = 0.9,
    readonly beta2 = 0.999,
    readonly eps = 1e-8,
  ) {
    this.m = new Float64Array(net.nParams);
    this.v = new Float64Array(net.nParams);
  }

  /** One update from the gradients accumulated over `n` samples. */
  step(n = 1): void {
    this.t++;
    const g = this.net.gradients();
    const theta = this.net.parameters();
    const c1 = 1 - Math.pow(this.beta1, this.t);
    const c2 = 1 - Math.pow(this.beta2, this.t);
    for (let i = 0; i < g.length; i++) {
      const gi = g[i] / n;
      this.m[i] = this.beta1 * this.m[i] + (1 - this.beta1) * gi;
      this.v[i] = this.beta2 * this.v[i] + (1 - this.beta2) * gi * gi;
      theta[i] -= (this.lr * (this.m[i] / c1)) / (Math.sqrt(this.v[i] / c2) + this.eps);
    }
    this.net.setParameters(theta);
    this.net.zeroGrad();
  }
}

/* -------------------------------------------------------------------------- */
/* Losses — each returns the loss and dL/dy for backward                        */
/* -------------------------------------------------------------------------- */

export interface LossGrad {
  loss: number;
  dy: Float64Array;
}

/** ½‖y − t‖² averaged over outputs. */
export function mseLoss(y: ArrayLike<number>, target: ArrayLike<number>): LossGrad {
  const n = y.length;
  const dy = new Float64Array(n);
  let loss = 0;
  for (let i = 0; i < n; i++) {
    const d = y[i] - target[i];
    loss += 0.5 * d * d;
    dy[i] = d / n;
  }
  return { loss: loss / n, dy };
}

/**
 * ½‖y − t‖² with over-estimates weighted `over` times more than under-estimates.
 * A heuristic's ε is a *maximum* of ĥ/h* − 1, so the training objective should
 * dislike guessing high more than guessing low; this is the simplest loss that
 * does, and the one the shipped heuristic is trained with.
 */
export function asymmetricMseLoss(y: ArrayLike<number>, target: ArrayLike<number>, over: number): LossGrad {
  const n = y.length;
  const dy = new Float64Array(n);
  let loss = 0;
  for (let i = 0; i < n; i++) {
    const d = y[i] - target[i];
    const w = d > 0 ? over : 1;
    loss += 0.5 * w * d * d;
    dy[i] = (w * d) / n;
  }
  return { loss: loss / n, dy };
}

export function softmax(z: ArrayLike<number>): Float64Array {
  let max = -Infinity;
  for (let i = 0; i < z.length; i++) if (z[i] > max) max = z[i];
  const out = new Float64Array(z.length);
  let s = 0;
  for (let i = 0; i < z.length; i++) {
    out[i] = Math.exp(z[i] - max);
    s += out[i];
  }
  for (let i = 0; i < z.length; i++) out[i] /= s;
  return out;
}

export function logSumExp(z: ArrayLike<number>): number {
  let max = -Infinity;
  for (let i = 0; i < z.length; i++) if (z[i] > max) max = z[i];
  let s = 0;
  for (let i = 0; i < z.length; i++) s += Math.exp(z[i] - max);
  return max + Math.log(s);
}

/**
 * −log softmax(z)[target]: the negative log-likelihood of one observed cell.
 * Averaged over a dataset this is H(p*) + D_KL(p* ‖ p̂_θ) (Derivation 2), which
 * is why "maximum likelihood" and "minimize KL to the path distribution" are
 * one objective.
 */
export function crossEntropyLoss(logits: ArrayLike<number>, target: number): LossGrad {
  const p = softmax(logits);
  const dy = new Float64Array(logits.length);
  for (let i = 0; i < dy.length; i++) dy[i] = p[i];
  dy[target] -= 1;
  return { loss: -Math.log(Math.max(p[target], 1e-300)), dy };
}

/* -------------------------------------------------------------------------- */
/* A mini-batch training loop                                                   */
/* -------------------------------------------------------------------------- */

export interface Sample {
  x: ArrayLike<number>;
  /** Regression target (for `mseLoss`) or the observed class (for `crossEntropyLoss`). */
  y: ArrayLike<number> | number;
}

export interface TrainOptions {
  epochs: number;
  batch: number;
  lr: number;
  rng: Rng;
  /** 'mse' for regression, 'xent' for a categorical output, 'asym' for regression that penalizes over-estimates `over`× more. */
  loss: 'mse' | 'xent' | 'asym';
  over?: number;
  /** Called after each epoch with the mean loss; return false to stop early. */
  onEpoch?: (epoch: number, meanLoss: number) => boolean | void;
  /** Multiply the learning rate by this after every epoch (1 = constant). */
  decay?: number;
}

/** Shuffled mini-batch Adam over `data`; returns the mean loss per epoch. */
export function trainMlp(net: Mlp, data: readonly Sample[], opts: TrainOptions): number[] {
  const adam = new Adam(net, opts.lr);
  const order = data.map((_, i) => i);
  const history: number[] = [];
  let lr = opts.lr;
  for (let e = 0; e < opts.epochs; e++) {
    // Fisher–Yates with the seeded generator so a training run is reproducible.
    for (let i = order.length - 1; i > 0; i--) {
      const j = Math.floor(opts.rng.next() * (i + 1));
      const t = order[i];
      order[i] = order[j];
      order[j] = t;
    }
    let total = 0;
    for (let b = 0; b < order.length; b += opts.batch) {
      const end = Math.min(order.length, b + opts.batch);
      for (let k = b; k < end; k++) {
        const s = data[order[k]];
        const y = net.forward(s.x, true);
        const lg =
          opts.loss === 'xent'
            ? crossEntropyLoss(y, s.y as number)
            : opts.loss === 'asym'
              ? asymmetricMseLoss(y, s.y as ArrayLike<number>, opts.over ?? 4)
              : mseLoss(y, s.y as ArrayLike<number>);
        total += lg.loss;
        net.backward(lg.dy);
      }
      adam.lr = lr;
      adam.step(end - b);
    }
    const mean = total / order.length;
    history.push(mean);
    lr *= opts.decay ?? 1;
    if (opts.onEpoch && opts.onEpoch(e, mean) === false) break;
  }
  return history;
}

/* -------------------------------------------------------------------------- */
/* Finite-difference gradient check                                            */
/* -------------------------------------------------------------------------- */

/**
 * Largest relative discrepancy between backprop and central differences on a
 * scalar loss — the one test that makes every later training claim credible.
 */
export function gradientCheck(net: Mlp, x: ArrayLike<number>, target: ArrayLike<number>, h = 1e-6): number {
  net.zeroGrad();
  const y = net.forward(x, true);
  net.backward(mseLoss(y, target).dy);
  const analytic = net.gradients();
  const theta = net.parameters();
  let worst = 0;
  for (let i = 0; i < theta.length; i++) {
    const save = theta[i];
    theta[i] = save + h;
    net.setParameters(theta);
    const lp = mseLoss(net.forward(x), target).loss;
    theta[i] = save - h;
    net.setParameters(theta);
    const lm = mseLoss(net.forward(x), target).loss;
    theta[i] = save;
    const numeric = (lp - lm) / (2 * h);
    const scale = Math.max(1e-6, Math.abs(numeric) + Math.abs(analytic[i]));
    worst = Math.max(worst, Math.abs(numeric - analytic[i]) / scale);
  }
  net.setParameters(theta);
  net.zeroGrad();
  return worst;
}
