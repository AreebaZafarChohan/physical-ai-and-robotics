import React, { useState } from 'react';
import { motion } from 'framer-motion';
import Button from './Button';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';

const Navbar: React.FC = () => {
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const { siteConfig } = useDocusaurusContext();
  const themeConfig: any = siteConfig.themeConfig;
  const navbar: { items: any[] } = themeConfig.navbar;
  const navItems = navbar.items;

  return (
    <header className="sticky top-0 z-50 bg-white backdrop-blur-sm border-b h-14 border-dark-card">
      <nav className="container mx-auto px-4 py-3 pt-4">
        <div className="flex items-center justify-between">
          {/* Logo */}
          <div className="flex items-center space-x-2">
            <img src={useBaseUrl('/img/logo.png')} alt="Logo" className="w-10 h-10" />
            <Link to={useBaseUrl('/')} className="text-xl font-bold text-white no-underline hover:no-underline">
              {(siteConfig.themeConfig as any).navbar.title}
            </Link>
          </div>

          {/* Desktop Navigation */}
          <div className="hidden md:flex items-center space-x-8">
            {navItems.map((item, index) => {
              if (item.type === 'docSidebar') {
                return (
                  <Link
                    key={index}
                    to={useBaseUrl('/docs/intro')} // Link to intro for docSidebar
                    className="text-white/80 hover:text-neon-purple-400 transition-colors duration-300"
                  >
                    {item.label}
                  </Link>
                );
              } else if (item.type === 'localeDropdown') {
                // Skip the locale dropdown here since it's handled by the default Docusaurus navbar
                return null;
              } else if (item.href) {
                return (
                  <a
                    key={index}
                    href={item.href}
                    className="text-white/80 hover:text-neon-purple-400 transition-colors duration-300"
                    target="_blank" // Open external links in new tab
                    rel="noopener noreferrer"
                  >
                    {item.label || item.name}
                  </a>
                );
              } else if (item.to) {
                return (
                  <Link
                    key={index}
                    to={useBaseUrl(item.to)}
                    className="text-white/80 hover:text-neon-purple-400 transition-colors duration-300"
                  >
                    {item.label || item.name}
                  </Link>
                );
              }
              return null;
            })}
          </div>

          {/* CTA Button and Additional Controls - Hidden on mobile until menu opens */}
          <div className="hidden md:flex items-center space-x-4">
            <Link to="/docs/intro">
              <Button variant="primary">Get Started</Button>
            </Link>
          </div>

          {/* Mobile Menu Button */}
          <button
            className="md:hidden text-white focus:outline-none"
            onClick={() => setIsMenuOpen(!isMenuOpen)}
          >
            <svg
              className="w-6 h-6"
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
              xmlns="http://www.w3.org/2000/svg"
            >
              {isMenuOpen ? (
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              ) : (
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
              )}
            </svg>
          </button>
        </div>

        {/* Mobile Menu */}
        {isMenuOpen && (
          <motion.div
            initial={{ opacity: 0, height: 0 }}
            animate={{ opacity: 1, height: 'auto' }}
            exit={{ opacity: 0, height: 0 }}
            className="md:hidden mt-4 pb-4"
          >
            <div className="flex flex-col space-y-4">
              {navItems.map((item, index) => {
                if (item.type === 'docSidebar') {
                  return (
                    <Link
                      key={index}
                      to={useBaseUrl('/docs/intro')} // Link to intro for docSidebar
                      className="text-white/80 hover:text-neon-purple-400 transition-colors duration-300 py-2 border-b border-dark-card/50"
                      onClick={() => setIsMenuOpen(false)}
                    >
                      {item.label}
                    </Link>
                  );
                } else if (item.type === 'localeDropdown') {
                  // Skip the locale dropdown here since it's handled by the default Docusaurus navbar
                  return null;
                } else if (item.href) {
                  return (
                    <a
                      key={index}
                      href={item.href}
                      className="text-white/80 hover:text-neon-purple-400 transition-colors duration-300 py-2 border-b border-dark-card/50"
                      target="_blank"
                      rel="noopener noreferrer"
                      onClick={() => setIsMenuOpen(false)}
                    >
                      {item.label || item.name}
                    </a>
                  );
                } else if (item.to) {
                  return (
                    <Link
                      key={index}
                      to={useBaseUrl(item.to)}
                      className="text-white/80 hover:text-neon-purple-400 transition-colors duration-300 py-2 border-b border-dark-card/50"
                      onClick={() => setIsMenuOpen(false)}
                    >
                      {item.label || item.name}
                    </Link>
                  );
                }
                return null;
              })}
              <div className="pt-4">
                <Link to="/docs/intro">
                  <Button variant="primary" fullWidth>Get Started</Button>
                </Link>
              </div>
            </div>
          </motion.div>
        )}
      </nav>
    </header>
  );
};

export default Navbar;