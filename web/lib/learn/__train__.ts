/**
 * Offline training + benchmark run — the TypeScript stand-in for
 * `cargo run --release --example learned_sampler`. Produces `weights.ts`
 * (the browser's safetensors) and `bench-data.ts` (the Board's recorded runs).
 *
 *   npx esbuild lib/learn/__train__.ts --bundle --platform=node --format=esm \
 *     --outfile=.next/train22.mjs --log-level=error && node .next/train22.mjs
 *
 * Everything here is seeded; rerunning reproduces the files bit for bit (up to
 * the wall-clock columns, which are what they are on the machine that ran it).
 */

import { writeFileSync } from 'node:fs';
import { Rng } from '../prob/rng';
import { Mlp, trainMlp } from './mlp';
import { SAMPLER_SIZES } from './sampler';
import { HEURISTIC_SIZES, measureEpsilon } from './heuristic';
import { POLICY_SIZES } from './neural-planner';
import { DENOISER_SIZES, denoise, trainDenoiser } from './diffusion';
import { generateHeuristicData, generatePathData, worldCases } from './data';
import { runBenchmark } from './bench';

const OUT_DIR = new URL('.', import.meta.url).pathname.replace(/\/\.next\/?$/, '/lib/learn/');
const log = (...a: unknown[]) => console.log(new Date().toISOString().slice(11, 19), ...a);
const now = () => performance.now();

const SEED = 7;
const N_TRAIN_WORLDS = 600;
const rng = new Rng(SEED);

log('worlds');
const cases = worldCases(1000, N_TRAIN_WORLDS, 'train');
const valCases = worldCases(5000, 16, 'train');
const heldCases = worldCases(20000, 16, 'heldout');

log('path data (PRM on every world)');
let t0 = now();
const paths = generatePathData(cases, 3, rng.fork());
const pathMs = now() - t0;
log(`  ${paths.solved}/${paths.attempted} queries solved · ${paths.sampler.length} sampler targets · ${paths.policy.length} policy targets · ${pathMs.toFixed(0)} ms`);

log('train sampler');
t0 = now();
const sampler = new Mlp(SAMPLER_SIZES, 'tanh', 'linear', rng.fork());
const samplerLoss = trainMlp(sampler, paths.sampler, { epochs: 50, batch: 32, lr: 2e-3, decay: 0.92, loss: 'xent', rng: rng.fork(), onEpoch: (e, l) => void (e % 5 === 4 && log(`  epoch ${e + 1} xent ${l.toFixed(4)}`)) });
const samplerMs = now() - t0;
log(`  ${samplerMs.toFixed(0)} ms · final ${samplerLoss[samplerLoss.length - 1].toFixed(4)} nats (uniform would be ${Math.log(144).toFixed(4)})`);

log('train policy');
t0 = now();
const policy = new Mlp(POLICY_SIZES, 'tanh', 'linear', rng.fork());
const policyLoss = trainMlp(policy, paths.policy, { epochs: 40, batch: 32, lr: 2e-3, decay: 0.92, loss: 'mse', rng: rng.fork(), onEpoch: (e, l) => void (e % 5 === 4 && log(`  epoch ${e + 1} mse ${l.toFixed(4)}`)) });
const policyMs = now() - t0;
log(`  ${policyMs.toFixed(0)} ms`);

log('heuristic data (backward Dijkstra on every world)');
t0 = now();
const heur = generateHeuristicData(cases.slice(0, 240), 3, 110, rng.fork());
log(`  ${heur.samples.length} labels from ${heur.goals} goals · ${(now() - t0).toFixed(0)} ms`);
log('train heuristic');
t0 = now();
const heuristic = new Mlp(HEURISTIC_SIZES, 'tanh', 'linear', rng.fork());
const heurLoss = trainMlp(heuristic, heur.samples, { epochs: 20, batch: 32, lr: 2e-3, decay: 0.9, loss: 'asym', over: 6, rng: rng.fork(), onEpoch: (e, l) => void (e % 5 === 4 && log(`  epoch ${e + 1} mse ${l.toFixed(5)}`)) });
const heuristicMs = now() - t0;
const epsVal = measureEpsilon(heuristic, valCases, 3, new Rng(99));
const epsHeld = measureEpsilon(heuristic, heldCases, 3, new Rng(99));
log(`  ${heuristicMs.toFixed(0)} ms · ε_val ${epsVal.epsilon.toFixed(4)} (${epsVal.violations}/${epsVal.audited} over) mean ĥ/h* ${epsVal.meanRatio.toFixed(3)} vs octile ${epsVal.meanOctileRatio.toFixed(3)} · ε_heldout ${epsHeld.epsilon.toFixed(4)}`);

log('train denoiser');
t0 = now();
const denoiser = new Mlp(DENOISER_SIZES, 'tanh', 'linear', rng.fork());
const den = trainDenoiser(denoiser, rng.fork(), 3000, 40);
const denoiserMs = now() - t0;
const plain = denoise(denoiser, new Rng(3), 64);
const guided = denoise(denoiser, new Rng(3), 64, { guidance: { smooth: 0.5, obstacle: 4.0, margin: 0.42 } });
log(`  ${denoiserMs.toFixed(0)} ms · loss ${den.losses[den.losses.length - 1].toFixed(4)} · valid ${plain.validFraction.toFixed(2)} (above ${plain.above}, below ${plain.below}) · guided ${guided.validFraction.toFixed(2)}`);

log('benchmark');
t0 = now();
const bench = runBenchmark(
  { sampler, policy },
  {
    worldsPerSuite: 30,
    seed: 70000,
    budgets: [100, 200, 400, 800, 1600, 3200, 6400, 12800],
    lambda: 0.1,
    trainingMs: { sampler: pathMs + samplerMs, policy: pathMs + policyMs },
    onProgress: (d, t) => void (d % 10 === 0 && log(`  ${d}/${t}`)),
  },
);
log(`  ${(now() - t0).toFixed(0)} ms`);

const r4 = (x: number) => (Number.isFinite(x) ? Number(x.toFixed(4)) : x);
const r2 = (x: number) => (Number.isFinite(x) ? Number(x.toFixed(2)) : x);
const benchOut = {
  ...bench,
  suites: Object.fromEntries(
    Object.entries(bench.suites).map(([k, s]) => [
      k,
      {
        gen: s.gen,
        queries: s.queries.map((q) => ({
          seed: q.seed,
          gen: q.gen,
          start: q.start.map(r4),
          goal: q.goal.map(r4),
          incumbent: r4(q.incumbent),
          runs: q.runs.map((r) => ({ ...r, cost: Number.isFinite(r.cost) ? r4(r.cost) : null, ms: r2(r.ms), inferenceMs: Number(r.inferenceMs.toFixed(3)) })),
        })),
      },
    ]),
  ),
  trainingMs: { sampler: Math.round(bench.trainingMs.sampler), policy: Math.round(bench.trainingMs.policy) },
};

const header = (what: string) => `/* GENERATED by lib/learn/__train__.ts (${what}) — seed ${SEED}, ${new Date().toISOString().slice(0, 10)}. Do not edit; rerun the script. */\n`;

const meta = {
  seed: SEED,
  trainWorlds: N_TRAIN_WORLDS,
  queriesPerWorld: 3,
  pathQueriesSolved: paths.solved,
  pathQueriesAttempted: paths.attempted,
  samplerTargets: paths.sampler.length,
  policyTargets: paths.policy.length,
  heuristicLabels: heur.samples.length,
  heuristicWorlds: 240,
  epochs: { sampler: 50, policy: 40, heuristic: 20, denoiser: 40 },
  heuristicLoss: 'asymmetric squared error, over-estimates weighted 6×',
  finalLoss: { sampler: r4(samplerLoss[samplerLoss.length - 1]), policy: r4(policyLoss[policyLoss.length - 1]), heuristic: r4(heurLoss[heurLoss.length - 1]), denoiser: r4(den.losses[den.losses.length - 1]) },
  ms: { pathData: Math.round(pathMs), sampler: Math.round(samplerMs), policy: Math.round(policyMs), heuristic: Math.round(heuristicMs), denoiser: Math.round(denoiserMs) },
  epsilon: {
    validation: { epsilon: r4(epsVal.epsilon), violations: epsVal.violations, audited: epsVal.audited, meanRatio: r4(epsVal.meanRatio), meanOctileRatio: r4(epsVal.meanOctileRatio), worlds: 16, goalsPerWorld: 3, seed0: 5000 },
    heldout: { epsilon: r4(epsHeld.epsilon), violations: epsHeld.violations, audited: epsHeld.audited, meanRatio: r4(epsHeld.meanRatio), meanOctileRatio: r4(epsHeld.meanOctileRatio), worlds: 16, goalsPerWorld: 3, seed0: 20000 },
  },
  denoiser: { validPlain: r4(plain.validFraction), validGuided: r4(guided.validFraction), above: plain.above, below: plain.below },
};

writeFileSync(
  OUT_DIR + 'weights.ts',
  header('weights') +
    `import type { MlpJson } from './mlp';\n\n` +
    `export const TRAINING_META = ${JSON.stringify(meta, null, 2)} as const;\n\n` +
    `export const SAMPLER_WEIGHTS: MlpJson = ${JSON.stringify(sampler.toJSON(5))};\n\n` +
    `export const HEURISTIC_WEIGHTS: MlpJson = ${JSON.stringify(heuristic.toJSON(5))};\n\n` +
    `export const POLICY_WEIGHTS: MlpJson = ${JSON.stringify(policy.toJSON(5))};\n\n` +
    `export const DENOISER_WEIGHTS: MlpJson = ${JSON.stringify(denoiser.toJSON(5))};\n`,
);
writeFileSync(
  OUT_DIR + 'bench-data.ts',
  header('benchmark') + `import type { BenchData } from './bench';\n\n` + `export const BENCH_DATA = ${JSON.stringify(benchOut)} as unknown as BenchData;\n`,
);
log('wrote weights.ts and bench-data.ts to', OUT_DIR);
