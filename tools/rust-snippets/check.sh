#!/usr/bin/env bash
# Verify every ```rust block in the book. Run from anywhere:
#
#   tools/rust-snippets/check.sh                 # everything (chapters + design docs)
#   tools/rust-snippets/check.sh --only ch13     # one chapter
#   tools/rust-snippets/check.sh --verbose       # list the tier every snippet reached
#   tools/rust-snippets/check.sh --help          # all options
#
# Exit status 0 = PASS. The Markdown report lands in tools/rust-snippets/target/REPORT.md.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$HOME/.cargo/bin:$PATH"
if ! command -v cargo >/dev/null 2>&1; then
  echo "cargo not found — install Rust from https://rustup.rs (stable, edition 2024 needs >= 1.85)" >&2
  exit 2
fi
exec cargo run --quiet --release --manifest-path "$here/Cargo.toml" -- "$@"
