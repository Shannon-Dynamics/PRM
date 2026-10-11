import { createMDX } from 'fumadocs-mdx/next';

const withMDX = createMDX();

/**
 * Where the book is mounted.
 *
 * On GitHub Pages a *project* site lives under `/<repo>/`, not at the root, so
 * every asset URL the export bakes in has to carry that prefix or the reader
 * gets a page of unstyled HTML and no simulations. CI passes the prefix in via
 * `NEXT_PUBLIC_BASE_PATH` (from `actions/configure-pages`, which also returns
 * an empty prefix for a user site or a custom domain). Locally it is unset, so
 * `npm run dev` and `npm run preview` keep serving from `/`.
 *
 * `configure-pages` reports "/" for a root-mounted site; Next rejects that as a
 * basePath, so normalize it — and any trailing slash — away.
 */
const basePath = (process.env.NEXT_PUBLIC_BASE_PATH ?? '').replace(/\/+$/, '');

/** @type {import('next').NextConfig} */
const config = {
  reactStrictMode: true,
  // The book is a static site: every chapter renders at build time, so readers
  // download HTML + the simulation bundles and nothing else.
  output: 'export',
  // Every page is emitted as <route>/index.html and linked with a trailing
  // slash, matching the canonical URLs and the sitemap.
  trailingSlash: true,
  basePath,
  // Rewrites `/_next/...` in the emitted HTML. Next derives this from basePath
  // anyway; stating it keeps the two from drifting if one is ever overridden.
  assetPrefix: basePath || undefined,
  images: { unoptimized: true },
  typescript: { ignoreBuildErrors: false },

  experimental: {
    /**
     * Bound the prerender workers.
     *
     * Vercel's basic build machine has 8 GB. Left to itself Next sizes the
     * worker pool from the host's core count, and each worker holds its own
     * copy of the chapter bundles — 23 chapters of MDX, KaTeX and simulation
     * components. Two is enough to overlap I/O without the pool being what
     * fills the box. Same setting as the sister book, for the same reason.
     */
    cpus: 2,
  },
};

export default withMDX(config);
