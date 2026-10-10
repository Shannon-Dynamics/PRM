/**
 * MDX smoke test: compile and server-render every chapter with the real
 * remark-math / rehype-katex pipeline and stub components, so that a stray
 * `{expr}` in prose (which MDX treats as a JavaScript expression) fails here
 * in seconds instead of minutes into `next build`.
 *
 *   npm run smoke            # all chapters
 *   npm run smoke -- ch11    # chapters whose file name contains "ch11"
 */
import { readdirSync, readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { evaluate } from '@mdx-js/mdx';
import * as runtime from 'react/jsx-runtime';
import { createElement } from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';
import { katexOptions } from '../lib/katex-macros.ts';

const WEB = join(dirname(fileURLToPath(import.meta.url)), '..');
const DIR = join(WEB, 'content', 'chapters');
const filter = process.argv[2] ?? '';
const files = readdirSync(DIR).filter((f) => f.endsWith('.mdx') && f.includes(filter)).sort();

// Every capitalized JSX tag in the file becomes a stub that renders its
// children (MDX spreads the components object, so it must be a plain object).
const Stub = (props) => createElement('div', null, props.children ?? null);
const stubsFor = (src) => {
  const names = new Set([...src.matchAll(/<([A-Z][A-Za-z0-9]*)/g)].map((m) => m[1]));
  return Object.fromEntries([...names].map((n) => [n, Stub]));
};

let failed = 0;
for (const f of files) {
  const src = readFileSync(join(DIR, f), 'utf8')
    .replace(/^---\n[\s\S]*?\n---\n/, '') // frontmatter
    .replace(/^import .*$/gm, ''); // imports resolve to stubs instead
  const components = stubsFor(src);
  try {
    const { default: Content } = await evaluate(src, {
      ...runtime,
      remarkPlugins: [remarkMath],
      rehypePlugins: [[rehypeKatex, katexOptions()]],
      useMDXComponents: () => components,
    });
    const html = renderToStaticMarkup(createElement(Content, { components }));
    const errors = (html.match(/katex-error/g) ?? []).length;
    if (errors) {
      failed++;
      console.log(`FAIL  ${f}  — ${errors} katex-error(s)`);
    } else {
      console.log(`PASS  ${f}  (${(html.length / 1024).toFixed(0)} kB)`);
    }
  } catch (e) {
    failed++;
    const msg = String(e?.message ?? e).split('\n')[0];
    console.log(`FAIL  ${f}  — ${msg}`);
  }
}
console.log(`\n${files.length - failed}/${files.length} chapters render`);
process.exit(failed ? 1 : 0);
