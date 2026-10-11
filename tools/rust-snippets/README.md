# rust-snippets — verify the book's Rust listings

Every ```rust block in the book — the 125 in the web chapters (`web/content/chapters/*.mdx`) and
the 23 in the design docs (`Chapter-NN.md`) — checked by the real Rust compiler against the book's
real crate stack.

```sh
tools/rust-snippets/check.sh              # from the repository root; exit 0 = PASS
cd web && npm run check:rust              # the same, next to the other book checks
```

The first run downloads and compiles the crate stack (a few minutes, mostly `candle`); after
that a full run takes about half a minute. The report is written to `target/REPORT.md`.

## What "verified" means

The listings are excerpts of a workspace the book describes but does not ship: a Chapter 13
listing calls `Manifold` from Chapter 5's crate, which exists only as prose and as the TypeScript
port the widgets run. So each listing is checked in tiers, and the report says which tier every
listing reached.

| tier | what it checks |
|---|---|
| **syntax** | The block parses as Rust (`syn`, full grammar). |
| **type-check** | The block is compiled by rustc as its own crate, edition 2024, depending on exactly the crates in `harness/Cargo.toml`, together with the items of the *other* listings of the same file (prose often shows a call before the definition it relies on). Errors that only say a name comes from another chapter's crate are *context*. Every other error — a wrong type, a method that does not exist on a nalgebra matrix, a path like `rand::rngs::Pcg64` that the real crate does not have, a reserved keyword — is a **finding** and fails the run. |
| **run** | A listing whose crate compiles with nothing unresolved has its `fn main` and `#[test]`s executed. When the book prints the program's output in a ```text block right after the listing, every printed line must appear in the real output. |

Today: 148/148 parse, 148/148 type-check with no findings, and Chapter 1's complete program runs
and prints exactly what the book shows.

## The elision conventions it understands

These are how the book omits code, so they are not findings:

- `fn f(..) -> T { /* … */ }` — a body that is only a comment becomes `unimplemented!()`, so the
  signature is still checked.
- `pub fn f(..) -> T;` inside an `impl` (or at module level) — an API listing; same treatment.
- An `impl` with no items, or whose braces contain `/* … */` or `…`, is deliberately partial:
  "missing trait items" there is not a finding. The same holds for a struct whose body is only a
  comment ("type parameter never used").
- `?` or `.await` in a statements-only example.

## Accepting a finding

A listing that is *meant* not to compile (the book's deliberate compile-error examples) goes in
`accepted.toml` with the error code and a reason. Entries that stop matching are reported as stale.

## Keeping it honest

- **The crate stack is `TOC.md §2` made executable.** `harness/Cargo.toml` pins nalgebra 0.35,
  rand 0.9 (+ `rand_pcg` 0.9, `rand_distr` 0.5), parry2d 0.30 (the f64 build), petgraph 0.8,
  pathfinding 4.15, ordered-float, proptest, candle 0.11. Change a version in the book → change it
  here → rerun.
- **Limitation:** a name the listing uses *without importing it* is indistinguishable from a name in
  another chapter's crate, so it is classed as context. That is how `Qbvh` slipped through until it
  was caught by hand (parry 0.26+ renamed it `Bvh`). Writing the `use` line in a listing is what
  lets the checker verify the name.
- **Limitation:** a call to a method the book never prints, on one of the book's own types, is
  context too — the listing is an excerpt of a larger `impl`.

## Layout

```
Cargo.toml, src/main.rs   the checker (syn + rustc via cargo)
harness/Cargo.toml        the crate stack; harness/gen/ is generated each run (one crate per listing)
accepted.toml             reviewed exceptions, each with a reason
check.sh                  the one-line entry point
target/REPORT.md          the last report (generated)
```
