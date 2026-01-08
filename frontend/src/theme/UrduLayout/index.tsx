import React from 'react';
import Layout from '@theme/Layout';
import { useLocation } from '@docusaurus/router';

interface UrduLayoutProps {
  children: React.ReactNode;
  title?: string;
  description?: string;
}

const UrduLayout: React.FC<UrduLayoutProps> = ({ children, title, description }) => {
  const location = useLocation();
  const isUrduLocale = location.pathname.startsWith('/ur/');

  // Apply RTL styling only for Urdu locale
  const layoutClass = isUrduLocale ? 'rtl' : '';

  return (
    <Layout title={title} description={description}>
      <div dir={isUrduLocale ? 'rtl' : 'ltr'} className={layoutClass}>
        {children}
      </div>
    </Layout>
  );
};

export default UrduLayout;