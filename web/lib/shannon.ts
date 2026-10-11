/**
 * Where the book sits relative to Shannon Dynamics.
 *
 * The book's navigation is its own; the only links back out are the wordmark,
 * the footer, and the PDF, and those are absolute cross-origin URLs to the
 * marketing site.
 */

const trimEnd = (s: string) => s.replace(/\/+$/, '');
const trimStart = (s: string) => s.replace(/^\/+/, '');

/**
 * The marketing site.
 *
 * Overridable so a staging deployment of the site can be linked instead of
 * production; unset — the normal case — means shannon.id.
 */
export const SHANNON_SITE = trimEnd(
  process.env.NEXT_PUBLIC_SHANNON_SITE || 'https://shannon.id',
);

/** A URL on the marketing site. */
export const site = (path: string) => `${SHANNON_SITE}/${trimStart(path)}`;

/**
 * The prefix the book is mounted under — `/<repo>` on GitHub Pages, empty
 * locally. Same normalization as next.config.mjs.
 */
const BASE_PATH = trimEnd(process.env.NEXT_PUBLIC_BASE_PATH ?? '');

/**
 * A URL for a file in `public/`.
 *
 * `basePath` rewrites what Next itself emits, but not a raw `<img src>`, so
 * every `public/` reference goes through here to carry the prefix.
 */
export const asset = (path: string) => `${BASE_PATH}/${trimStart(path)}`;

export const SHANNON_HOME = site('');
export const SHANNON_CAPABILITIES = site('#capabilities');
export const SHANNON_PRODUCTS = site('#showcase');
export const SHANNON_LIBRARY = site('#library');
export const SHANNON_ECOSYSTEM = site('#ecosystem');
export const SHANNON_CONTACT = site('#contact');
export const SHANNON_BOOKS = site('library-books.html');
export const SHANNON_EMAIL = 'hello@shannon.id';

/** This book's own front door — what the in-book navigation points home to. */
export const BOOK_HOME = '/';

/** The book's title, as the breadcrumb and the mobile bar say it. */
export const BOOK_TITLE = 'PRINCIPLES OF ROBOT MOTION';

/**
 * The PDF edition.
 *
 * The file itself is published by the marketing site, so this stays an absolute
 * URL to it rather than a copy of a multi-megabyte asset in this deployment.
 */
export const BOOK_PDF = site('book-pdf/principles-of-robot-motion-via-rust.pdf');

/** The Library sections, for the footer's column back to the site. */
export const LIBRARY_LINKS = [
  {
    href: SHANNON_BOOKS,
    title: 'Books',
    blurb: 'Technical books and guides',
  },
  {
    href: site('library-open-source.html'),
    title: 'Open Source',
    blurb: 'Repositories, guides and API reference',
  },
  {
    href: site('library-publications.html'),
    title: 'Publications',
    blurb: 'Papers, preprints and field notes',
  },
];
