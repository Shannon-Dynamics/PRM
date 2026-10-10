/**
 * A binary min-heap on lexicographic numeric keys — the priority queue behind
 * every search in this chapter.
 *
 * Keys are short arrays so a search can say what it means by "ties": A* sorts
 * on [f, h, order], D* Lite on [k₁, k₂]. Entries are never deleted or
 * decreased in place; a search that improves a node pushes a fresh entry and
 * discards the stale one when it surfaces (the "lazy deletion" every textbook
 * A* quietly relies on). That keeps the heap tiny and the search code honest
 * about when a node is really expanded.
 */

export interface HeapEntry<T> {
  key: readonly number[];
  value: T;
  /** Monotone insertion counter: the final tie-break, so runs are reproducible. */
  seq: number;
}

export function compareKeys(a: readonly number[], b: readonly number[]): number {
  const n = Math.min(a.length, b.length);
  for (let i = 0; i < n; i++) {
    if (a[i] < b[i]) return -1;
    if (a[i] > b[i]) return 1;
  }
  return a.length - b.length;
}

/**
 * Lexicographic comparison that treats components within `eps` as equal. Keys
 * built from sums of √2 steps tie in exact arithmetic and differ by an ulp in
 * floating point; a termination test that depends on the tie must not see it.
 */
export function compareKeysEps(a: readonly number[], b: readonly number[], eps = 1e-9): number {
  const n = Math.min(a.length, b.length);
  for (let i = 0; i < n; i++) {
    if (a[i] < b[i] - eps) return -1;
    if (a[i] > b[i] + eps) return 1;
  }
  return a.length - b.length;
}

export class KeyedHeap<T> {
  private readonly items: HeapEntry<T>[] = [];
  private counter = 0;

  get size(): number {
    return this.items.length;
  }

  push(key: readonly number[], value: T): void {
    const e: HeapEntry<T> = { key, value, seq: this.counter++ };
    this.items.push(e);
    let i = this.items.length - 1;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (this.less(this.items[parent], e)) break;
      this.items[i] = this.items[parent];
      i = parent;
    }
    this.items[i] = e;
  }

  peek(): HeapEntry<T> | undefined {
    return this.items[0];
  }

  pop(): HeapEntry<T> | undefined {
    const items = this.items;
    if (items.length === 0) return undefined;
    const top = items[0];
    const last = items.pop()!;
    if (items.length > 0) {
      let i = 0;
      const n = items.length;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = -1;
        if (l < n && this.less(items[l], last)) m = l;
        if (r < n && this.less(items[r], m === -1 ? last : items[l])) m = r;
        if (m === -1) break;
        items[i] = items[m];
        i = m;
      }
      items[i] = last;
    }
    return top;
  }

  clear(): void {
    this.items.length = 0;
  }

  private less(a: HeapEntry<T>, b: HeapEntry<T>): boolean {
    const c = compareKeys(a.key, b.key);
    return c < 0 || (c === 0 && a.seq < b.seq);
  }
}
