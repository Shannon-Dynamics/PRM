/**
 * The graph a planner searches — Choset App. H.1 as an interface.
 *
 * Every planner in Parts II–III ends by handing a graph to a search. The graph
 * may be a grid (this chapter), a visibility graph or Voronoi diagram (Ch. 8),
 * a cell adjacency graph (Ch. 10), or a sampled roadmap (Ch. 11), and the
 * search must not care. So the interface is the smallest thing best-first
 * search needs: `neighbors(n)` is Choset's Star(n) with the edge costs
 * c(n, x) attached, and `key(n)` is what the open and closed sets hash on.
 * Costs are non-negative throughout — Dijkstra's correctness depends on it.
 */

export interface Edge<N> {
  to: N;
  /** c(n, x) ≥ 0 — the weight of the edge, in whatever units the planner optimizes. */
  cost: number;
}

export interface Graph<N> {
  /** Star(n): the nodes adjacent to n, with the cost of reaching each. */
  neighbors(n: N): readonly Edge<N>[];
  /** A hashable identity for n, so g, parent, open and closed can be maps. */
  key(n: N): number | string;
  /**
   * Optional deterministic tie-break among equal priorities (e.g. lexicographic
   * cell order on a grid). Without it ties fall to insertion order.
   */
  order?(n: N): number;
}

/**
 * A small explicit graph for the hand-traceable examples — Choset's lettered
 * graphs and the non-optimistic counterexample. Undirected by default: adding
 * (a, b) adds (b, a) too, since Choset's edges are undirected unless drawn otherwise.
 */
export class ExplicitGraph implements Graph<string> {
  private readonly adj = new Map<string, Edge<string>[]>();

  addNode(n: string): this {
    if (!this.adj.has(n)) this.adj.set(n, []);
    return this;
  }

  addEdge(a: string, b: string, cost: number, directed = false): this {
    if (cost < 0) throw new Error(`ExplicitGraph: negative edge cost ${cost} on ${a}→${b}`);
    this.addNode(a).addNode(b);
    this.adj.get(a)!.push({ to: b, cost });
    if (!directed) this.adj.get(b)!.push({ to: a, cost });
    return this;
  }

  nodes(): string[] {
    return [...this.adj.keys()];
  }

  neighbors(n: string): readonly Edge<string>[] {
    return this.adj.get(n) ?? [];
  }

  key(n: string): string {
    return n;
  }
}

export interface LabeledSearchProblem {
  graph: ExplicitGraph;
  start: string;
  goal: string;
  /** h(n), as drawn inside Choset's node icons. */
  h: Record<string, number>;
  /** The true cost-to-go h*(n), for the text's comparison. */
  hStar: Record<string, number>;
}

/**
 * Choset §H.2.5, figure H.20 — the non-optimistic counterexample, with the
 * numbers the design doc fixes. Edges S–A 4, A–G 4, S–B 3, B–G 2; heuristic
 * h(S) = 2, h(A) = 3, h(B) = 10, h(G) = 0. Since h(B) = 10 > h*(B) = 2, A*
 * pops G via A at f = 8 before it ever expands B at f = 13, and returns the
 * cost-8 path S, A, G although S, B, G costs 5.
 */
export const NONOPTIMISTIC: LabeledSearchProblem = {
  graph: new ExplicitGraph().addEdge('S', 'A', 4).addEdge('A', 'G', 4).addEdge('S', 'B', 3).addEdge('B', 'G', 2),
  start: 'S',
  goal: 'G',
  h: { S: 2, A: 3, B: 10, G: 0 },
  hStar: { S: 5, A: 4, B: 2, G: 0 },
};

/**
 * A graph with the same shape whose heuristic *is* admissible (h(B) = 2 = h*(B)),
 * so the widget and the exercises can show A* recovering the optimum the
 * moment the one bad estimate is fixed.
 */
export const OPTIMISTIC_TWIN: LabeledSearchProblem = {
  ...NONOPTIMISTIC,
  h: { S: 2, A: 3, B: 2, G: 0 },
};
