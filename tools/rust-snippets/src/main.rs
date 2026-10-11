//! Verify every ```rust block in *Principles of Robot Motion via Rust*.
//!
//! The book's listings are excerpts of a workspace that is described, not
//! shipped: a snippet in Chapter 13 calls `Manifold` from Chapter 5's crate,
//! which exists only as prose and as the TypeScript port the widgets run. So
//! "does it compile?" is asked in tiers, and the report says which tier every
//! snippet reached:
//!
//!  1. syntax — the block parses as Rust (`syn`, full grammar).
//!  2. type-check — the block is compiled by rustc, as its own crate, against
//!     the book's real crate stack (`harness/Cargo.toml`), together with the
//!     items of the other snippets of the same file. Errors that only say
//!     "this name lives in another chapter's crate" are context, not defects.
//!     Every other error is a finding.
//!  3. run — a snippet whose crate compiles with nothing unresolved has its
//!     `fn main` and `#[test]`s executed; when the book prints the expected
//!     output right after the listing, the real output is compared with it.
//!
//! Conventions the checker understands (they are how the book elides code):
//!  * a function whose body is only a comment, `{ /* … */ }`, is elided — the
//!    body becomes `unimplemented!()` so the signature is still checked;
//!  * a signature ending in `;` inside an `impl` (or at module level) is an
//!    API listing — it gets the same treatment;
//!  * an `impl` block with no items is elided — "missing trait items" there is
//!    not a finding.
//!
//! Findings that are intentional (the book's deliberate compile-error
//! listings) or reviewed false positives live in `accepted.toml`, each with a
//! reason. An accepted entry that no longer matches anything is reported, so
//! the list cannot rot.
//!
//! Usage (from the repository root):
//!   tools/rust-snippets/check.sh [OPTIONS]
//! Options:
//!   --only <substr>    check only files whose path contains <substr> (ch13, Chapter-05, …)
//!   --chapters         only the web chapters (web/content/chapters/*.mdx)
//!   --design-docs      only the design docs (Chapter-NN.md)
//!   --no-run           skip tier 3
//!   --verbose          list every snippet with the tier it reached
//!   --report <path>    where to write the Markdown report
//!                      (default: tools/rust-snippets/target/REPORT.md)

use proc_macro2::{LineColumn, Span, TokenStream};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, Stdio};
use std::time::{Duration, Instant};
use syn::parse::{Parse, ParseStream};
use syn::visit::Visit;

// ---------------------------------------------------------------------------
// Snippets
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Source {
    Chapter,
    Design,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    /// A sequence of items (structs, fns, impls, uses …): compiled as a crate.
    Items,
    /// Statements (a usage example): compiled as the body of a function.
    Statements,
}

struct Snippet {
    /// Path relative to the book root.
    file: String,
    /// 1-based position among the file's rust blocks.
    index: usize,
    title: Option<String>,
    /// 1-based line, in `file`, of the block's first code line.
    md_line: usize,
    code: String,
    /// The ```text block printed right after the listing, if any.
    expected_output: Option<String>,
    source: Source,
    /// Crate name in the harness workspace, e.g. `c13_3`, `d13_1`.
    krate: String,
    prepared: Result<Prepared, SyntaxError>,
}

#[derive(Debug)]
struct SyntaxError {
    md_line: usize,
    message: String,
}

struct Prepared {
    kind: Kind,
    /// The text compiled for this snippet (edited code; for statements, `{ … }`).
    body: String,
    /// Elided bodies the checker filled in.
    elided: usize,
    /// Line ranges (1-based, in the snippet) of empty `impl` blocks.
    empty_impls: Vec<(usize, usize)>,
    /// Top-level items, for later snippets of the same file to see.
    items: Vec<TopItem>,
    /// Names this snippet defines or imports at module level.
    defines: BTreeSet<String>,
    has_main: bool,
    has_tests: bool,
}

#[derive(Clone)]
struct TopItem {
    key: ItemKey,
    text: String,
}

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
enum ItemKey {
    /// struct / enum / trait / fn / const / type / mod … by name.
    Named(String),
    /// `impl <trait> for <type>` or `impl <type>`, as normalized tokens.
    Impl(String),
    /// One imported name (`use a::b::C;` → `C`).
    Use { binds: Option<String>, text: String },
}

impl Snippet {
    fn label(&self) -> String {
        match &self.title {
            Some(t) => format!("{}#{} ({})", short(&self.file), self.index, t),
            None => format!("{}#{}", short(&self.file), self.index),
        }
    }
}

fn short(file: &str) -> &str {
    file.rsplit('/').next().unwrap_or(file)
}

/// Strip up to `indent` leading whitespace characters.
fn dedent(line: &str, indent: usize) -> &str {
    let cut = line.char_indices().take_while(|(k, c)| *k < indent && c.is_whitespace()).count();
    &line[line.char_indices().nth(cut).map_or(line.len(), |(b, _)| b)..]
}

/// Every ```rust fence in a Markdown/MDX file.
fn extract(root: &Path, rel: &str, source: Source, group: usize) -> Vec<Snippet> {
    let text = fs::read_to_string(root.join(rel)).unwrap_or_default();
    let lines: Vec<&str> = text.lines().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < lines.len() {
        let trimmed = lines[i].trim_start();
        let Some(meta) = trimmed.strip_prefix("```rust") else {
            i += 1;
            continue;
        };
        let indent = lines[i].len() - trimmed.len();
        let mut j = i + 1;
        let mut body = Vec::new();
        while j < lines.len() && lines[j].trim() != "```" {
            body.push(dedent(lines[j], indent));
            j += 1;
        }
        // The book prints a program's output in a ```text block right after it.
        let mut expected_output = None;
        let mut k = j + 1;
        while k < lines.len() && lines[k].trim().is_empty() {
            k += 1;
        }
        if k < lines.len() && lines[k].trim_start().starts_with("```text") {
            let ind = lines[k].len() - lines[k].trim_start().len();
            let mut e = Vec::new();
            let mut m = k + 1;
            while m < lines.len() && lines[m].trim() != "```" {
                e.push(dedent(lines[m], ind));
                m += 1;
            }
            expected_output = Some(e.join("\n"));
        }
        let index = out.len() + 1;
        let title = meta.split("title=\"").nth(1).and_then(|r| r.split('"').next()).map(str::to_owned);
        let prefix = if source == Source::Chapter { 'c' } else { 'd' };
        out.push(Snippet {
            file: rel.to_owned(),
            index,
            title,
            md_line: i + 2,
            code: body.join("\n"),
            expected_output,
            source,
            krate: format!("{prefix}{group:02}_{index}"),
            prepared: Err(SyntaxError { md_line: 0, message: String::new() }),
        });
        i = j + 1;
    }
    out
}

// ---------------------------------------------------------------------------
// Tier 1: parse, and fill in elisions
// ---------------------------------------------------------------------------

fn line_starts(src: &str) -> Vec<usize> {
    std::iter::once(0).chain(src.match_indices('\n').map(|(i, _)| i + 1)).collect()
}

/// Byte offset of a span position in `src`.
fn offset(line_starts: &[usize], src: &str, lc: LineColumn) -> usize {
    let start = line_starts[lc.line - 1];
    let line = &src[start..];
    start + line.char_indices().nth(lc.column).map_or(line.len(), |(b, _)| b)
}

/// `fn sig;` — an API listing line.
struct BodylessFn {
    semi: Span,
}

impl Parse for BodylessFn {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        input.call(syn::Attribute::parse_outer)?;
        let _: syn::Visibility = input.parse()?;
        let _: Option<syn::Token![default]> = input.parse()?;
        let _: syn::Signature = input.parse()?;
        let semi: syn::Token![;] = input.parse()?;
        if !input.is_empty() {
            return Err(input.error("trailing tokens"));
        }
        Ok(BodylessFn { semi: semi.span })
    }
}

struct Elider {
    edits: Vec<(LineColumn, LineColumn, &'static str)>,
    /// Line ranges of deliberately partial items: an `impl` with no items or with an
    /// elision comment, a struct whose body is only a comment.
    empty_impls: Vec<(usize, usize)>,
    has_tests: bool,
    /// The parsed text, to read the comments syn drops.
    src: String,
}

/// Does the text between these braces mark something as omitted?
fn marks_elision(text: &str) -> bool {
    text.contains("/*") || text.contains('…') || text.contains("...")
}

fn returns_value(sig: &syn::Signature) -> bool {
    match &sig.output {
        syn::ReturnType::Default => false,
        syn::ReturnType::Type(_, ty) => !matches!(&**ty, syn::Type::Tuple(t) if t.elems.is_empty()),
    }
}

impl Elider {
    fn new(src: &str) -> Self {
        Elider { edits: Vec::new(), empty_impls: Vec::new(), has_tests: false, src: src.to_owned() }
    }
    fn text(&self, span: Span) -> &str {
        let ls = line_starts(&self.src);
        let (a, b) = (offset(&ls, &self.src, span.start()), offset(&ls, &self.src, span.end()));
        &self.src[a.min(b)..b]
    }
    fn body(&mut self, sig: &syn::Signature, block: &syn::Block) {
        if block.stmts.is_empty() && returns_value(sig) {
            let span = block.brace_token.span.join();
            self.edits.push((span.start(), span.end(), "{ unimplemented!() }"));
        }
    }
    fn verbatim(&mut self, ts: &TokenStream) {
        if let Ok(b) = syn::parse2::<BodylessFn>(ts.clone()) {
            self.edits.push((b.semi.start(), b.semi.end(), " { unimplemented!() }"));
        }
    }
}

impl<'ast> Visit<'ast> for Elider {
    fn visit_item_fn(&mut self, f: &'ast syn::ItemFn) {
        if f.attrs.iter().any(|a| a.path().is_ident("test")) {
            self.has_tests = true;
        }
        self.body(&f.sig, &f.block);
        syn::visit::visit_item_fn(self, f);
    }
    fn visit_impl_item_fn(&mut self, f: &'ast syn::ImplItemFn) {
        self.body(&f.sig, &f.block);
        syn::visit::visit_impl_item_fn(self, f);
    }
    fn visit_trait_item_fn(&mut self, f: &'ast syn::TraitItemFn) {
        if let Some(b) = &f.default {
            self.body(&f.sig, b);
        }
        syn::visit::visit_trait_item_fn(self, f);
    }
    fn visit_impl_item(&mut self, i: &'ast syn::ImplItem) {
        if let syn::ImplItem::Verbatim(ts) = i {
            self.verbatim(ts);
        }
        syn::visit::visit_impl_item(self, i);
    }
    fn visit_item(&mut self, i: &'ast syn::Item) {
        match i {
            syn::Item::Verbatim(ts) => self.verbatim(ts),
            syn::Item::Impl(imp) => {
                let s = imp.brace_token.span.join();
                if imp.items.is_empty() || marks_elision(self.text(s)) {
                    self.empty_impls.push((imp.impl_token.span.start().line, s.end().line));
                }
            }
            syn::Item::Struct(st) if st.fields.is_empty() => {
                let s = syn::spanned::Spanned::span(st);
                if marks_elision(self.text(s)) {
                    self.empty_impls.push((s.start().line, s.end().line));
                }
            }
            syn::Item::Macro(m) if m.mac.path.is_ident("proptest") => self.has_tests = true,
            _ => {}
        }
        syn::visit::visit_item(self, i);
    }
}

/// Apply the elider's edits, keeping every line where it was.
fn apply(src: &str, mut edits: Vec<(LineColumn, LineColumn, &'static str)>) -> String {
    let ls = line_starts(src);
    edits.sort_by_key(|e| std::cmp::Reverse((e.0.line, e.0.column)));
    let mut out = src.to_owned();
    for (a, b, with) in edits {
        let (x, y) = (offset(&ls, src, a), offset(&ls, src, b));
        let newlines = src[x..y].matches('\n').count();
        out.replace_range(x..y, &format!("{with}{}", "\n".repeat(newlines)));
    }
    out
}

fn use_leaves(tree: &syn::UseTree, prefix: &mut Vec<String>, out: &mut Vec<(Option<String>, String)>) {
    match tree {
        syn::UseTree::Path(p) => {
            prefix.push(p.ident.to_string());
            use_leaves(&p.tree, prefix, out);
            prefix.pop();
        }
        syn::UseTree::Name(n) => {
            let name = n.ident.to_string();
            if name == "self" {
                out.push((prefix.last().cloned(), format!("use {};", prefix.join("::"))));
            } else {
                let path = prefix.iter().cloned().chain([name.clone()]).collect::<Vec<_>>().join("::");
                out.push((Some(name), format!("use {path};")));
            }
        }
        syn::UseTree::Rename(r) => {
            let path = prefix.iter().cloned().chain([r.ident.to_string()]).collect::<Vec<_>>().join("::");
            out.push((Some(r.rename.to_string()), format!("use {path} as {};", r.rename)));
        }
        syn::UseTree::Glob(_) => out.push((None, format!("use {}::*;", prefix.join("::")))),
        syn::UseTree::Group(g) => {
            for t in &g.items {
                use_leaves(t, prefix, out);
            }
        }
    }
}

fn item_name(item: &syn::Item) -> Option<String> {
    Some(match item {
        syn::Item::Const(i) => i.ident.to_string(),
        syn::Item::Enum(i) => i.ident.to_string(),
        syn::Item::Fn(i) => i.sig.ident.to_string(),
        syn::Item::Mod(i) => i.ident.to_string(),
        syn::Item::Static(i) => i.ident.to_string(),
        syn::Item::Struct(i) => i.ident.to_string(),
        syn::Item::Trait(i) => i.ident.to_string(),
        syn::Item::TraitAlias(i) => i.ident.to_string(),
        syn::Item::Type(i) => i.ident.to_string(),
        syn::Item::Union(i) => i.ident.to_string(),
        syn::Item::Macro(m) if m.mac.path.is_ident("macro_rules") => m.ident.as_ref()?.to_string(),
        _ => return None,
    })
}

/// Top-level items worth carrying forward to later snippets of the same file.
fn top_items(file: &syn::File, src: &str) -> (Vec<TopItem>, BTreeSet<String>) {
    let ls = line_starts(src);
    let mut items = Vec::new();
    let mut defines = BTreeSet::new();
    for item in &file.items {
        let span = syn::spanned::Spanned::span(item);
        let text = || src[offset(&ls, src, span.start())..offset(&ls, src, span.end())].to_owned();
        match item {
            syn::Item::Use(u) => {
                let mut leaves = Vec::new();
                use_leaves(&u.tree, &mut Vec::new(), &mut leaves);
                for (binds, text) in leaves {
                    if let Some(b) = &binds {
                        defines.insert(b.clone());
                    }
                    items.push(TopItem { key: ItemKey::Use { binds, text: text.clone() }, text });
                }
            }
            syn::Item::Impl(imp) => {
                let tr = imp.trait_.as_ref().map(|(_, p, _)| quote::quote!(#p).to_string());
                let ty = &imp.self_ty;
                let key = format!("{} for {}", tr.unwrap_or_default(), quote::quote!(#ty));
                items.push(TopItem { key: ItemKey::Impl(key), text: text() });
            }
            _ => {
                let Some(name) = item_name(item) else { continue };
                defines.insert(name.clone());
                let is_test_mod = matches!(item, syn::Item::Mod(m)
                    if m.ident == "tests" || m.attrs.iter().any(|a| a.path().is_ident("cfg")));
                if name == "main" || is_test_mod {
                    continue; // never inherited: one `main` and one `tests` per snippet
                }
                items.push(TopItem { key: ItemKey::Named(name), text: text() });
            }
        }
    }
    (items, defines)
}

fn prepare(code: &str) -> Result<Prepared, (usize, String)> {
    match syn::parse_file(code) {
        Ok(file) => {
            let mut el = Elider::new(code);
            el.visit_file(&file);
            let elided = el.edits.len();
            let body = apply(code, el.edits);
            let reparsed = syn::parse_file(&body).map_err(|e| (e.span().start().line, format!("after filling elisions: {e}")))?;
            let (items, defines) = top_items(&reparsed, &body);
            let has_main = reparsed.items.iter().any(|i| matches!(i, syn::Item::Fn(f) if f.sig.ident == "main"));
            Ok(Prepared { kind: Kind::Items, body, elided, empty_impls: el.empty_impls, items, defines, has_main, has_tests: el.has_tests })
        }
        Err(as_items) => {
            let wrapped = format!("{{\n{code}\n}}");
            match syn::parse_str::<syn::Block>(&wrapped) {
                Ok(block) => {
                    let mut el = Elider::new(&wrapped);
                    el.visit_block(&block);
                    let elided = el.edits.len();
                    let body = apply(&wrapped, el.edits);
                    let empty_impls = el.empty_impls.iter().map(|&(a, b)| (a - 1, b - 1)).collect();
                    Ok(Prepared { kind: Kind::Statements, body, elided, empty_impls, items: Vec::new(), defines: BTreeSet::new(), has_main: false, has_tests: false })
                }
                Err(as_block) => {
                    // Report whichever reading got further into the snippet.
                    let (l1, l2) = (as_items.span().start().line, as_block.span().start().line.saturating_sub(1));
                    if l2 > l1 { Err((l2.max(1), as_block.to_string())) } else { Err((l1.max(1), as_items.to_string())) }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tier 2: compile each snippet as its own crate
// ---------------------------------------------------------------------------

const HEADER: &str = "#![allow(unused, dead_code, non_snake_case, non_camel_case_types, non_upper_case_globals, unreachable_code, unused_must_use)]\n";

/// Lines of generated text that precede the snippet's first code line.
fn prefix_lines(kind: Kind) -> usize {
    match kind {
        Kind::Items => 1,
        Kind::Statements => 2, // header, then `fn __snippet() {`
    }
}

/// The crate root for snippet `k`: the snippet, then the items of the other
/// snippets of the same file that it does not redefine. Returns the text, the
/// last generated line that belongs to the snippet itself, and every type or
/// item name the crate defines.
fn crate_text(snippets: &[Snippet], k: usize) -> (String, usize, HashSet<String>) {
    let s = &snippets[k];
    let p = s.prepared.as_ref().unwrap();
    let mut text = String::from(HEADER);
    match p.kind {
        Kind::Items => text.push_str(&p.body),
        Kind::Statements => {
            text.push_str("fn __snippet() ");
            text.push_str(&p.body);
        }
    }
    text.push('\n');
    let snippet_end = text.lines().count();

    // Carry forward earlier snippets of the same file, latest definition first.
    let mut taken_names: HashSet<String> = p.defines.iter().cloned().collect();
    let mut taken_impls: HashSet<String> = HashSet::new();
    let mut taken_uses: HashSet<String> = HashSet::new();
    for it in &p.items {
        if let ItemKey::Impl(key) = &it.key {
            taken_impls.insert(key.clone());
        }
    }
    let mut prelude: Vec<String> = Vec::new();
    // Earlier snippets first (nearest wins a name clash), then later ones:
    // prose often shows a call before the definition it relies on.
    let others = snippets[..k].iter().rev().chain(snippets[k + 1..].iter());
    for e in others.filter(|e| e.file == s.file) {
        let Ok(ep) = &e.prepared else { continue };
        if ep.kind != Kind::Items {
            continue;
        }
        for it in &ep.items {
            let keep = match &it.key {
                ItemKey::Named(n) => taken_names.insert(n.clone()),
                ItemKey::Impl(key) => taken_impls.insert(key.clone()),
                ItemKey::Use { binds: Some(b), text } => taken_uses.insert(text.clone()) && taken_names.insert(b.clone()),
                ItemKey::Use { binds: None, text } => taken_uses.insert(text.clone()),
            };
            if keep {
                prelude.push(it.text.clone());
            }
        }
    }
    if !prelude.is_empty() {
        text.push_str("\n// ---- context: the other snippets of this file ----\n");
        for item in prelude {
            text.push_str(&item);
            text.push('\n');
        }
    }
    if p.has_main {
        // Tier 3 runs `main` through the test harness so its output is captured.
        text.push_str("\n#[cfg(test)]\nmod __book_main {\n    #[test]\n    fn main() { let _ = super::main(); }\n}\n");
    }
    (text, snippet_end, taken_names)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Class {
    /// A name from another chapter's crate (or a later snippet) is not in scope.
    Context,
    /// An artifact of the elision conventions (empty impl, `?` in an example).
    Elision,
    /// Anything else: a real defect, or an intentional one listed in accepted.toml.
    Finding,
}

#[derive(Clone)]
struct Diag {
    krate: String,
    code: Option<String>,
    message: String,
    rendered: String,
    gen_line: usize,
}

/// Crates the harness really has (plus the standard library).
const DEPS: &[&str] = &[
    "std", "core", "alloc", "nalgebra", "rand", "rand_distr", "rand_pcg", "parry2d", "petgraph", "pathfinding",
    "ordered_float", "proptest", "candle_core", "candle_nn",
];

/// Does the message name a path inside a crate the harness really has? Then an
/// unresolved name is a real mistake (`rand::rngs::Pcg64` does not exist), not
/// a reference to another chapter.
fn names_real_crate(message: &str) -> bool {
    let quoted_hit = message.split('`').skip(1).step_by(2).any(|q| {
        let root = q.trim_start_matches("::").split("::").next().unwrap_or("");
        DEPS.contains(&root) && q.contains("::")
    });
    quoted_hit || DEPS.iter().any(|d| message.contains(&format!("in `{d}`")) || message.contains(&format!("in crate `{d}`")))
}

/// The type a "no method named … found for …" message is about, reduced to its
/// base name: `&mut Explorer<'_>` → `Explorer`.
fn receiver_base(msg: &str) -> Option<String> {
    let ty = msg.rsplit('`').nth(1)?;
    let mut ty = ty.trim_start_matches('&').trim_start_matches("mut ").trim();
    // `Box<dyn VectorField<N>>`, `Box<(dyn T + 'static)>`, `Rc<T>`: the book's type is inside.
    for wrapper in ["Box<", "Rc<", "Arc<"] {
        if let Some(inner) = ty.strip_prefix(wrapper) {
            ty = inner.trim_start_matches('(').trim_start_matches("dyn ").trim();
        }
    }
    let ty = ty.trim_start_matches("dyn ");
    let base = ty.split(['<', ' ', ':', '+', ')']).next()?.to_owned();
    (!base.is_empty()).then_some(base)
}

fn classify(d: &Diag, p: &Prepared, snippet_line: usize, defined: &HashSet<String>, has_context: bool) -> Class {
    let code = d.code.as_deref().unwrap_or("");
    let msg = d.message.as_str();
    // A method or associated function the book does not print, on one of the
    // book's own types: the listing is an excerpt of a larger impl.
    if code == "E0599" && (msg.starts_with("no method named") || msg.starts_with("no function or associated item named") || msg.starts_with("no variant or associated item named")) {
        if receiver_base(msg).is_some_and(|b| defined.contains(&b)) {
            return Class::Context;
        }
    }
    // Inference and associated-type errors that follow from a name that did not
    // resolve (a trait from another chapter bounds `M`, so `M::Point` is unknown).
    if has_context && matches!(code, "E0282" | "E0283" | "E0220" | "E0223") {
        return Class::Context;
    }
    const UNRESOLVED: &[&str] = &["E0405", "E0412", "E0422", "E0425", "E0432", "E0433", "E0531", "E0532", "E0574", "E0576", "E0583"];
    let unresolved = UNRESOLVED.contains(&code)
        || msg.starts_with("cannot find macro")
        || msg.starts_with("cannot find derive macro")
        || msg.starts_with("cannot find attribute");
    if unresolved {
        return if names_real_crate(msg) { Class::Finding } else { Class::Context };
    }
    if matches!(code, "E0046" | "E0392") && p.empty_impls.iter().any(|&(a, b)| snippet_line >= a && snippet_line <= b) {
        return Class::Elision;
    }
    if code == "E0277" && p.kind == Kind::Statements && msg.contains("the `?` operator can only be used") {
        return Class::Elision;
    }
    if code == "E0728" && p.kind == Kind::Statements {
        return Class::Elision; // `.await` in an example body
    }
    Class::Finding
}

/// Run cargo in `dir`, killing it after `limit`.
fn cargo(dir: &Path, target: &Path, args: &[&str], limit: Duration) -> (String, String, bool) {
    let mut child = Command::new("cargo")
        .args(args)
        .current_dir(dir)
        .env("CARGO_TARGET_DIR", target)
        .env("CARGO_TERM_COLOR", "never")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to run cargo — is the Rust toolchain installed? (https://rustup.rs)");
    // Drain both pipes on threads so a chatty build cannot deadlock.
    let mut so = child.stdout.take().unwrap();
    let mut se = child.stderr.take().unwrap();
    let ho = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = std::io::Read::read_to_string(&mut so, &mut s);
        s
    });
    let he = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = std::io::Read::read_to_string(&mut se, &mut s);
        s
    });
    let start = Instant::now();
    let status = loop {
        if let Some(st) = child.try_wait().unwrap() {
            break Some(st);
        }
        if start.elapsed() > limit {
            let _ = child.kill();
            let _ = child.wait();
            break None;
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    let (out, mut err) = (ho.join().unwrap(), he.join().unwrap());
    if status.is_none() {
        err.push_str(&format!("\n(timed out after {}s)", limit.as_secs()));
    }
    (out, err, status.is_some_and(|s| s.success()))
}

fn diagnostics(stdout: &str) -> Vec<Diag> {
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for line in stdout.lines() {
        let Ok(v) = serde_json::from_str::<serde_json::Value>(line) else { continue };
        if v["reason"] != "compiler-message" || v["message"]["level"] != "error" {
            continue;
        }
        let m = &v["message"];
        let spans = m["spans"].as_array().cloned().unwrap_or_default();
        let Some(primary) = spans.iter().find(|s| s["is_primary"] == true).or(spans.first()) else { continue };
        let krate = v["target"]["name"].as_str().unwrap_or("").to_owned();
        // Paths are relative to the workspace root: `gen/<crate>/src/lib.rs`.
        if primary["file_name"] != format!("gen/{krate}/src/lib.rs").as_str() {
            continue;
        }
        let gen_line = primary["line_start"].as_u64().unwrap_or(0) as usize;
        let message = m["message"].as_str().unwrap_or("").to_owned();
        // `cargo check --tests` builds each crate twice (lib, lib test): keep one copy.
        if !seen.insert((krate.clone(), gen_line, message.clone())) {
            continue;
        }
        out.push(Diag {
            krate,
            code: m["code"]["code"].as_str().map(str::to_owned),
            message,
            rendered: m["rendered"].as_str().unwrap_or("").to_owned(),
            gen_line,
        });
    }
    out
}

/// Workspace dependency names, read from the harness manifest.
fn workspace_deps(manifest: &str) -> Vec<String> {
    let v: toml::Value = toml::from_str(manifest).expect("harness/Cargo.toml does not parse");
    v["workspace"]["dependencies"].as_table().map(|t| t.keys().cloned().collect()).unwrap_or_default()
}

fn write_workspace(harness: &Path, deps: &[String], crates: &[(String, String)]) {
    let gen_dir = harness.join("gen");
    let _ = fs::remove_dir_all(&gen_dir);
    let deps_toml: String = deps.iter().map(|d| format!("{d}.workspace = true\n")).collect();
    for (name, text) in crates {
        let dir = gen_dir.join(name);
        fs::create_dir_all(dir.join("src")).unwrap();
        fs::write(
            dir.join("Cargo.toml"),
            format!("# Generated by tools/rust-snippets — do not edit.\n[package]\nname = \"{name}\"\nversion = \"0.0.0\"\nedition = \"2024\"\npublish = false\n\n[lib]\npath = \"src/lib.rs\"\ndoctest = false\n\n[dependencies]\n{deps_toml}"),
        )
        .unwrap();
        fs::write(dir.join("src/lib.rs"), text).unwrap();
    }
}

// ---------------------------------------------------------------------------
// Tier 3: compare a run with the output the book prints
// ---------------------------------------------------------------------------

fn norm(line: &str) -> String {
    line.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// Lines of the expected output that the run did not print.
fn missing_lines(expected: &str, actual: &str) -> Vec<String> {
    let have: HashSet<String> = actual.lines().map(norm).collect();
    expected.lines().map(norm).filter(|l| !l.is_empty() && !have.contains(l)).collect()
}

// ---------------------------------------------------------------------------
// Accepted findings
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct AcceptedFile {
    #[serde(default)]
    accept: Vec<Accept>,
}

#[derive(Deserialize)]
struct Accept {
    /// File name or a suffix of its path, e.g. `ch04-configuration-space-i.mdx`.
    file: String,
    /// Snippet title (the fence's `title="…"`), or `index` (1-based) when it has none.
    title: Option<String>,
    index: Option<usize>,
    /// rustc error code (`E0308`), `syntax`, `output`, or `run`.
    code: String,
    /// Optional substring the error message must contain.
    contains: Option<String>,
    /// Why this is not a defect. Required.
    reason: String,
}

impl Accept {
    fn snippet(&self, s: &Snippet) -> bool {
        s.file.ends_with(&self.file)
            && self.title.as_ref().is_none_or(|t| s.title.as_deref() == Some(t.as_str()))
            && self.index.is_none_or(|i| i == s.index)
    }
    fn matches(&self, s: &Snippet, code: &str, message: &str) -> bool {
        self.snippet(s) && self.code == code && self.contains.as_ref().is_none_or(|c| message.contains(c.as_str()))
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Outcome {
    findings: Vec<(String, usize, String)>, // (code, md line, rendered)
    accepted: Vec<(String, usize, String)>, // (code, md line, reason)
    context: usize,
    elision: usize,
    /// Errors anywhere in the crate, inherited context included.
    crate_errors: usize,
    ran: Option<RunResult>,
}

enum RunResult {
    Passed { output_checked: Option<usize> },
    OutputDiffers(Vec<String>),
    Failed(String),
}

fn main() -> ExitCode {
    let tool_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let root = tool_dir.join("../..").canonicalize().expect("book root");
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |f: &str| args.iter().any(|a| a == f);
    let opt = |f: &str| args.iter().position(|a| a == f).and_then(|i| args.get(i + 1)).cloned();
    if flag("--help") || flag("-h") {
        let doc: Vec<&str> = include_str!("main.rs").lines().take_while(|l| l.starts_with("//!")).map(|l| l.trim_start_matches("//!").strip_prefix(' ').unwrap_or("")).collect();
        println!("{}", doc.join("\n"));
        return ExitCode::SUCCESS;
    }
    let only = opt("--only");
    let report_path = opt("--report").map(PathBuf::from).unwrap_or_else(|| tool_dir.join("target/REPORT.md"));
    let (want_chapters, want_docs) = match (flag("--chapters"), flag("--design-docs")) {
        (false, false) => (true, true),
        (c, d) => (c, d),
    };

    // ---- collect ----------------------------------------------------------
    let mut files: Vec<(String, Source, usize)> = Vec::new();
    if want_chapters {
        let mut names: Vec<String> = fs::read_dir(root.join("web/content/chapters"))
            .map(|rd| rd.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().into_owned()).filter(|n| n.starts_with("ch") && n.ends_with(".mdx")).collect())
            .unwrap_or_default();
        names.sort();
        for n in names {
            let num = n.get(2..4).and_then(|x| x.parse().ok()).unwrap_or(0);
            files.push((format!("web/content/chapters/{n}"), Source::Chapter, num));
        }
    }
    if want_docs {
        for n in 1..=99 {
            let rel = format!("Chapter-{n:02}.md");
            if root.join(&rel).exists() {
                files.push((rel, Source::Design, n));
            }
        }
    }
    if let Some(o) = &only {
        files.retain(|(f, _, _)| f.contains(o.as_str()));
    }
    let mut snippets: Vec<Snippet> = files.iter().flat_map(|(f, s, g)| extract(&root, f, *s, *g)).collect();
    if snippets.is_empty() {
        eprintln!("no ```rust blocks found");
        return ExitCode::FAILURE;
    }

    // ---- tier 1 -----------------------------------------------------------
    for s in &mut snippets {
        s.prepared = prepare(&s.code).map_err(|(line, message)| SyntaxError { md_line: s.md_line + line - 1, message });
    }

    // ---- tier 2 -----------------------------------------------------------
    let harness = tool_dir.join("harness");
    let target = tool_dir.join("target/harness");
    let manifest = fs::read_to_string(harness.join("Cargo.toml")).expect("harness/Cargo.toml");
    let deps = workspace_deps(&manifest);
    let mut crates = Vec::new();
    let mut snippet_end: BTreeMap<String, usize> = BTreeMap::new();
    let mut defined: BTreeMap<String, HashSet<String>> = BTreeMap::new();
    for k in 0..snippets.len() {
        if snippets[k].prepared.is_ok() {
            let (text, end, names) = crate_text(&snippets, k);
            snippet_end.insert(snippets[k].krate.clone(), end);
            defined.insert(snippets[k].krate.clone(), names);
            crates.push((snippets[k].krate.clone(), text));
        }
    }
    write_workspace(&harness, &deps, &crates);
    eprintln!("type-checking {} snippets, one crate each, against tools/rust-snippets/harness/Cargo.toml …", crates.len());
    let (stdout, stderr, _) = cargo(&harness, &target, &["check", "--workspace", "--tests", "--keep-going", "--message-format=json", "--quiet"], Duration::from_secs(3600));
    if !stdout.contains("\"reason\":\"compiler-message\"") && !stdout.contains("\"reason\":\"compiler-artifact\"") && !stdout.contains("build-finished") {
        eprintln!("cargo could not build the harness (dependency resolution or network?):\n{stderr}");
        return ExitCode::FAILURE;
    }

    let accepted_cfg: AcceptedFile = match fs::read_to_string(tool_dir.join("accepted.toml")) {
        Ok(t) => toml::from_str(&t).unwrap_or_else(|e| panic!("accepted.toml does not parse: {e}")),
        Err(_) => AcceptedFile { accept: Vec::new() },
    };
    let mut accept_used = vec![false; accepted_cfg.accept.len()];
    let mut accept = |s: &Snippet, code: &str, message: &str| -> Option<String> {
        let i = accepted_cfg.accept.iter().position(|a| a.matches(s, code, message))?;
        accept_used[i] = true;
        Some(accepted_cfg.accept[i].reason.clone())
    };

    let mut outcomes: BTreeMap<String, Outcome> = snippets.iter().map(|s| (s.krate.clone(), Outcome::default())).collect();
    let by_crate: BTreeMap<&str, usize> = snippets.iter().enumerate().map(|(i, s)| (s.krate.as_str(), i)).collect();
    let diags = diagnostics(&stdout);
    // A crate "has context" when some name in it comes from another chapter.
    let mut has_context: HashSet<String> = HashSet::new();
    for d in &diags {
        let Some(&si) = by_crate.get(d.krate.as_str()) else { continue };
        let p = snippets[si].prepared.as_ref().unwrap();
        if classify(d, p, 0, &defined[&d.krate], false) == Class::Context {
            has_context.insert(d.krate.clone());
        }
    }
    for d in diags {
        let Some(&si) = by_crate.get(d.krate.as_str()) else { continue };
        let s = &snippets[si];
        let p = s.prepared.as_ref().unwrap();
        let o = outcomes.get_mut(&d.krate).unwrap();
        o.crate_errors += 1;
        let first = prefix_lines(p.kind) + 1;
        if d.gen_line < first || d.gen_line > snippet_end[&d.krate] {
            continue; // inherited context: reported against the snippet that owns it
        }
        let code_line = (d.gen_line - first + 1).min(s.code.lines().count().max(1));
        let md_line = s.md_line + code_line - 1;
        let code = d.code.clone().unwrap_or_else(|| "error".into());
        match classify(&d, p, code_line, &defined[&d.krate], has_context.contains(&d.krate)) {
            Class::Context => o.context += 1,
            Class::Elision => o.elision += 1,
            Class::Finding => match accept(s, &code, &d.message) {
                Some(why) => o.accepted.push((code, md_line, why)),
                None => o.findings.push((code, md_line, d.rendered.trim_end().to_owned())),
            },
        }
    }
    // Syntax errors can be accepted too (none are, today).
    let mut syntax_accepted = HashSet::new();
    for s in &snippets {
        if let Err(e) = &s.prepared {
            if let Some(why) = accept(s, "syntax", &e.message) {
                syntax_accepted.insert(s.krate.clone());
                outcomes.get_mut(&s.krate).unwrap().accepted.push(("syntax".into(), e.md_line, why));
            }
        }
    }

    // ---- tier 3 -----------------------------------------------------------
    let runnable: Vec<usize> = (0..snippets.len())
        .filter(|&i| {
            let s = &snippets[i];
            let Ok(p) = &s.prepared else { return false };
            outcomes[&s.krate].crate_errors == 0 && p.elided == 0 && (p.has_main || p.has_tests)
        })
        .collect();
    if !flag("--no-run") && !runnable.is_empty() {
        eprintln!("running {} self-contained snippets …", runnable.len());
        for &i in &runnable {
            let s = &snippets[i];
            let (out, err, ok) = cargo(&harness, &target, &["test", "-p", &s.krate, "--lib", "--quiet", "--", "--nocapture", "--test-threads=1"], Duration::from_secs(600));
            let result = if !ok {
                let log = format!("{out}\n{err}");
                let lines: Vec<&str> = log.lines().filter(|l| !l.trim().is_empty()).collect();
                RunResult::Failed(lines[lines.len().saturating_sub(25)..].join("\n"))
            } else {
                match (&s.expected_output, s.prepared.as_ref().unwrap().has_main) {
                    (Some(expected), true) => {
                        let missing = missing_lines(expected, &out);
                        if missing.is_empty() {
                            RunResult::Passed { output_checked: Some(expected.lines().filter(|l| !l.trim().is_empty()).count()) }
                        } else if accept(s, "output", &missing.join("\n")).is_some() {
                            RunResult::Passed { output_checked: None }
                        } else {
                            RunResult::OutputDiffers(missing)
                        }
                    }
                    _ => RunResult::Passed { output_checked: None },
                }
            };
            outcomes.get_mut(&s.krate).unwrap().ran = Some(result);
        }
    }

    // ---- report -----------------------------------------------------------
    let chapter_count = snippets.iter().filter(|s| s.source == Source::Chapter).count();
    let doc_count = snippets.len() - chapter_count;
    let (mut syntax_ok, mut syntax_bad) = (0, 0);
    let (mut clean, mut context_only, mut with_findings) = (0, 0, 0);
    let (mut ran_ok, mut ran_bad, mut outputs_ok) = (0, 0, 0);
    let mut details = String::new();
    let mut listing = String::new();
    for s in &snippets {
        let o = &outcomes[&s.krate];
        let tier = match &s.prepared {
            Err(e) => {
                if syntax_accepted.contains(&s.krate) {
                    syntax_ok += 1;
                    "syntax error (accepted)".to_string()
                } else {
                    syntax_bad += 1;
                    let _ = writeln!(details, "### ✗ syntax — {}\n\n`{}:{}` — {}\n", s.label(), s.file, e.md_line, e.message);
                    "SYNTAX ERROR".to_string()
                }
            }
            Ok(p) => {
                syntax_ok += 1;
                if !o.findings.is_empty() {
                    with_findings += 1;
                    let _ = writeln!(details, "### ✗ {} finding(s) — {}\n", o.findings.len(), s.label());
                    for (code, line, rendered) in &o.findings {
                        let _ = writeln!(details, "`{}:{}` {}\n\n```text\n{}\n```\n", s.file, line, code, rendered);
                    }
                } else if o.context > 0 || o.elision > 0 {
                    context_only += 1;
                } else {
                    clean += 1;
                }
                let mut t = if !o.findings.is_empty() {
                    format!("{} FINDING(S)", o.findings.len())
                } else if o.context > 0 || o.elision > 0 {
                    format!("type-checked; {} name(s) from other chapters", o.context + o.elision)
                } else {
                    "type-checked, every name resolved".to_string()
                };
                if p.elided > 0 {
                    let _ = write!(t, "; {} elided bod{}", p.elided, if p.elided == 1 { "y" } else { "ies" });
                }
                if !o.accepted.is_empty() {
                    let _ = write!(t, "; {} accepted", o.accepted.len());
                }
                match &o.ran {
                    Some(RunResult::Passed { output_checked }) => {
                        ran_ok += 1;
                        t.push_str("; RAN OK");
                        if let Some(n) = output_checked {
                            outputs_ok += 1;
                            let _ = write!(t, ", output matches the book ({n} lines)");
                        }
                    }
                    Some(RunResult::OutputDiffers(missing)) => {
                        ran_bad += 1;
                        t.push_str("; RAN, OUTPUT DIFFERS");
                        let _ = writeln!(details, "### ✗ output differs from the book — {}\n\nLines the book prints that the program did not:\n\n```text\n{}\n```\n", s.label(), missing.join("\n"));
                    }
                    Some(RunResult::Failed(log)) => {
                        ran_bad += 1;
                        t.push_str("; RUN FAILED");
                        let _ = writeln!(details, "### ✗ run failed — {}\n\n```text\n{log}\n```\n", s.label());
                    }
                    None => {}
                }
                t
            }
        };
        let _ = writeln!(listing, "| `{}` | {} | {} |", s.file, s.index, tier.replace('|', "/"));
        if flag("--verbose") {
            println!("{:<64} {tier}", s.label());
        }
    }
    let stale: Vec<&Accept> = accepted_cfg.accept.iter().zip(&accept_used).filter(|(_, used)| !**used).map(|(a, _)| a).collect();
    let accepted_total: usize = outcomes.values().map(|o| o.accepted.len()).sum();
    let runnable_count = if flag("--no-run") { 0 } else { runnable.len() };
    let rows = [
        ("parsed as Rust", format!("{syntax_ok} / {}", snippets.len())),
        ("type-checked, every name resolved", clean.to_string()),
        ("type-checked, names from other chapters' crates (expected)", context_only.to_string()),
        ("with findings", with_findings.to_string()),
        ("executed (fn main / #[test])", format!("{ran_ok} of {runnable_count} passed")),
        ("output identical to what the book prints", outputs_ok.to_string()),
        ("accepted findings (accepted.toml)", accepted_total.to_string()),
    ];
    let mut md = String::new();
    let _ = writeln!(md, "# Rust snippet verification\n\nGenerated by `tools/rust-snippets`. {} snippets: {chapter_count} in the web chapters, {doc_count} in the design docs.\n", snippets.len());
    let _ = writeln!(md, "| tier | snippets |\n|---|---|");
    for (k, v) in &rows {
        let _ = writeln!(md, "| {k} | {v} |");
    }
    md.push('\n');
    if !stale.is_empty() {
        let _ = writeln!(md, "## Stale entries in accepted.toml\n");
        for a in &stale {
            let _ = writeln!(md, "- `{}` {} `{}` — matched nothing; remove it.", a.file, a.title.as_deref().unwrap_or(""), a.code);
        }
        md.push('\n');
    }
    if !details.is_empty() {
        let _ = writeln!(md, "## Problems\n\n{details}");
    }
    let accepted_list: Vec<String> = snippets.iter().flat_map(|s| outcomes[&s.krate].accepted.iter().map(move |(code, line, why)| format!("- `{}:{}` {} — {}", s.file, line, code, why))).collect();
    if !accepted_list.is_empty() {
        let _ = writeln!(md, "## Accepted (intentional or reviewed)\n\n{}\n", accepted_list.join("\n"));
    }
    let _ = writeln!(md, "## Every snippet\n\n| file | # | tier reached |\n|---|---|---|\n{listing}");
    if let Some(dir) = report_path.parent() {
        let _ = fs::create_dir_all(dir);
    }
    let _ = fs::write(&report_path, &md);

    println!("\nRust snippet verification — {} snippets ({chapter_count} chapter, {doc_count} design doc)\n", snippets.len());
    for (k, v) in &rows {
        println!("  {k:<62} {v}");
    }
    if !details.is_empty() {
        println!("\n{}", details.replace("### ", "").replace("```text\n", "").replace("```\n", ""));
    }
    for a in &stale {
        println!("stale accepted.toml entry (matched nothing): {} {} {}", a.file, a.title.as_deref().unwrap_or(""), a.code);
    }
    println!("\nreport: {}", report_path.display());
    if syntax_bad + with_findings + ran_bad + stale.len() > 0 {
        println!("RESULT: FAIL");
        ExitCode::FAILURE
    } else {
        println!("RESULT: PASS");
        ExitCode::SUCCESS
    }
}
