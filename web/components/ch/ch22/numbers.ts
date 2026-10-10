/**
 * Numbers the Chapter 22 prose quotes, computed from the library at render
 * time so the text can never drift from the shipped weights and recorded runs.
 * Everything here is a string ready to drop into a sentence.
 */

import { BENCH, TRAINING_META, fourCellExample, summarize, threeNodeExample, type PlannerName } from '@/lib/learn';

const pct = (x: number) => `${Math.round(100 * x)}%`;
const f = (x: number, d: number) => x.toFixed(d);

const row = (suite: 'train' | 'heldout', budget: number, planner: PlannerName) => summarize(BENCH.suites[suite], budget).find((r) => r.planner === planner)!;

const four = fourCellExample(0.1);
const three = threeNodeExample();

const B = BENCH.budgets;
const bLow = B[Math.min(2, B.length - 1)]; // e.g. 400 checks
const bMid = B[Math.min(4, B.length - 1)]; // e.g. 1600 checks
const bMax = BENCH.maxBudget;

export const N = {
  // §3(a) / §3(b)
  klHatU: f(four.klHatUniformNats, 4),
  klHatUBits: f(four.klHatUniformBits, 3),
  klStarHat: f(four.klTargetHat, 4),
  klStarU: f(four.klTargetUniform, 4),
  aimRatio: f(four.aimRatio, 0),
  mixture: four.mixture.map((m) => f(m, 3)).join(', '),
  mixtureMin: f(four.mixtureMin, 3),
  mixtureFloor: f(four.mixtureFloor, 3),
  eps3: f(three.epsilon, 4),
  bound3: f(three.bound, 3),

  // training
  trainWorlds: String(TRAINING_META.trainWorlds),
  pathsSolved: String(TRAINING_META.pathQueriesSolved),
  pathsAttempted: String(TRAINING_META.pathQueriesAttempted),
  samplerTargets: TRAINING_META.samplerTargets.toLocaleString('en-US'),
  policyTargets: TRAINING_META.policyTargets.toLocaleString('en-US'),
  heuristicLabels: TRAINING_META.heuristicLabels.toLocaleString('en-US'),
  heuristicWorlds: String(TRAINING_META.heuristicWorlds),
  samplerXent: f(TRAINING_META.finalLoss.sampler, 3),
  uniformXent: f(Math.log(144), 3),
  samplerEpochs: String(TRAINING_META.epochs.sampler),
  samplerSeconds: f((TRAINING_META.ms.pathData + TRAINING_META.ms.sampler) / 1000, 0),
  samplerTrainOnlySeconds: f(TRAINING_META.ms.sampler / 1000, 0),
  pathDataSeconds: f(TRAINING_META.ms.pathData / 1000, 0),
  epsVal: f(TRAINING_META.epsilon.validation.epsilon, 3),
  epsValViolations: TRAINING_META.epsilon.validation.violations.toLocaleString('en-US'),
  epsValAudited: TRAINING_META.epsilon.validation.audited.toLocaleString('en-US'),
  epsValWorlds: String(TRAINING_META.epsilon.validation.worlds),
  epsValMeanRatio: f(TRAINING_META.epsilon.validation.meanRatio, 3),
  epsValOctileRatio: f(TRAINING_META.epsilon.validation.meanOctileRatio, 3),
  epsHeld: f(TRAINING_META.epsilon.heldout.epsilon, 3),
  denoiserValid: pct(TRAINING_META.denoiser.validPlain),
  denoiserGuided: pct(TRAINING_META.denoiser.validGuided),

  // the Board
  worldsPerSuite: String(BENCH.meta.worldsPerSuite),
  budgets: B.join(', '),
  bLow: String(bLow),
  bMid: String(bMid),
  bMax: bMax.toLocaleString('en-US'),
  lambda: f(BENCH.lambda, 1),
  trainLearnedLow: pct(row('train', bLow, 'learned').success),
  trainUniformLow: pct(row('train', bLow, 'uniform').success),
  trainBridgeLow: pct(row('train', bLow, 'bridge').success),
  trainRrtLow: pct(row('train', bLow, 'rrt-connect').success),
  trainMpnetLow: pct(row('train', bLow, 'mpnet').success),
  heldLearnedLow: pct(row('heldout', bLow, 'learned').success),
  heldUniformLow: pct(row('heldout', bLow, 'uniform').success),
  heldBridgeLow: pct(row('heldout', bLow, 'bridge').success),
  heldRrtLow: pct(row('heldout', bLow, 'rrt-connect').success),
  heldMpnetLow: pct(row('heldout', bLow, 'mpnet').success),
  trainLearnedMid: pct(row('train', bMid, 'learned').success),
  trainUniformMid: pct(row('train', bMid, 'uniform').success),
  heldLearnedMid: pct(row('heldout', bMid, 'learned').success),
  heldUniformMid: pct(row('heldout', bMid, 'uniform').success),
  trainLearnedMax: pct(row('train', bMax, 'learned').success),
  trainUniformMax: pct(row('train', bMax, 'uniform').success),
  heldLearnedMax: pct(row('heldout', bMax, 'learned').success),
  heldUniformMax: pct(row('heldout', bMax, 'uniform').success),
  trainMpnetMax: pct(row('train', bMax, 'mpnet').success),
  heldMpnetMax: pct(row('heldout', bMax, 'mpnet').success),
  trainLearnedRatio: f(row('train', bMax, 'learned').costRatio, 2),
  trainUniformRatio: f(row('train', bMax, 'uniform').costRatio, 2),
  trainRrtRatio: f(row('train', bMax, 'rrt-connect').costRatio, 2),
  heldLearnedRatio: f(row('heldout', bMax, 'learned').costRatio, 2),
  heldUniformRatio: f(row('heldout', bMax, 'uniform').costRatio, 2),
  learnedInferenceMs: f(row('train', bMax, 'learned').inferenceMs, 2),
  mpnetInferenceMs: f(row('train', bMax, 'mpnet').inferenceMs, 1),
  learnedMs: f(row('train', bMax, 'learned').ms, 0),
  uniformMs: f(row('train', bMax, 'uniform').ms, 0),
  samplerTrainingSeconds: f(BENCH.trainingMs.sampler / 1000, 0),
  policyTrainingSeconds: f(BENCH.trainingMs.policy / 1000, 0),
  breakEven: (() => {
    const l = row('train', bMax, 'learned');
    const u = row('train', bMax, 'uniform');
    const gain = u.ms - l.ms;
    return gain > 0 ? Math.ceil(BENCH.trainingMs.sampler / gain).toLocaleString('en-US') : 'never (the learned row is not faster per query)';
  })(),
};

export default N;
