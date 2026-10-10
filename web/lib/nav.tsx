import type { BaseLayoutProps } from 'fumadocs-ui/layouts/shared';
import type { BookLink } from '@/components/shannon/site-header';

/**
 * The book's own routes — the whole of the Shannon top bar's navigation.
 * `Overview` is added by the bar itself, since every book has one.
 */
export const BOOK_LINKS: BookLink[] = [
  { label: 'Chapters', href: '/chapters' },
  { label: 'Notation', href: '/notation', exact: true },
];

/**
 * What is left of Fumadocs' own chrome.
 *
 * The site bar above it carries the branding and the navigation, so Fumadocs
 * keeps only its mobile sub-bar — the one that holds the sidebar trigger — and
 * that bar just needs to say which book you are in.
 */
export const bookNav: BaseLayoutProps = {
  nav: {
    title: <span className="text-[0.9rem] font-semibold tracking-tight">Robot Motion</span>,
    url: '/',
  },
};
