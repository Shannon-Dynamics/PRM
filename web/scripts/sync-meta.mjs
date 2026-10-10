/**
 * Regenerate content/chapters/meta.json from book-structure.ts, keeping only
 * the chapters whose MDX file exists. Runs before every build (`prebuild`), so
 * a partially written book always builds and the sidebar never links to a
 * chapter that is not there yet.
 */
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const WEB = join(dirname(fileURLToPath(import.meta.url)), '..');
const structure = readFileSync(join(WEB, 'lib', 'book-structure.ts'), 'utf8');
const SHORT = {
  'PART I': 'Part I · Foundations',
  'PART II': 'Part II · Classical Planners',
  'PART III': 'Part III · Sampling-Based Planning',
  'PART IV': 'Part IV · Planning with Uncertainty',
  'PART V': 'Part V · Dynamics and Constraints',
  'PART VI': 'Part VI · Frontiers',
};
const pages = ['index'];
let missing = 0;
for (const part of structure.split(/\n\s*\{\s*\n\s*id:/).slice(1)) {
  const id = part.match(/^\s*'([^']+)'/)[1];
  const slugs = [...part.matchAll(/slug:\s*'([^']+)'/g)].map((m) => m[1]);
  const present = slugs.filter((s) => existsSync(join(WEB, 'content', 'chapters', `${s}.mdx`)));
  missing += slugs.length - present.length;
  if (present.length) pages.push(`---${SHORT[id] ?? id}---`, ...present);
}
writeFileSync(
  join(WEB, 'content', 'chapters', 'meta.json'),
  JSON.stringify({ title: 'Chapters', root: true, pages }, null, 2) + '\n',
);
console.log(`meta.json: ${pages.filter((p) => !p.startsWith('---')).length - 1} chapters listed, ${missing} not yet written`);
