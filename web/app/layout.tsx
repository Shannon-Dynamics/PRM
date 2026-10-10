// The Shannon Dynamics pair, self-hosted through Fontsource.
//
// NOT `next/font/google`: that fetches the woff2 files from fonts.gstatic.com
// during the build, so a runner that cannot reach Google fails the deploy
// outright rather than degrading. Fontsource ships the same files through npm,
// so the only network the build needs is the registry it already depends on.
//
// `index.css` is the weight-axis variable face across all subsets; the Greek
// one earns its place in a book written in θ, ρ and τ.
import '@fontsource-variable/inter';
import '@fontsource-variable/jetbrains-mono';
import './global.css';
import type { Metadata } from 'next';
import { RootProvider } from 'fumadocs-ui/provider/next';
import { SiteFooter } from '@/components/shannon/site-footer';

export const metadata: Metadata = {
  title: {
    default: 'Principles of Robot Motion via Rust',
    template: '%s · Principles of Robot Motion via Rust',
  },
  description:
    'An interactive web book on robot motion planning: rigorous mathematical foundations, ' +
    'interactive simulations for every hard idea, and implementations in Rust.',
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className="flex min-h-screen flex-col">
        <RootProvider>
          {children}
          <SiteFooter />
        </RootProvider>
      </body>
    </html>
  );
}
